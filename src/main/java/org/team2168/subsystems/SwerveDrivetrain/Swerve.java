package org.team2168.subsystems.SwerveDrivetrain;

import static edu.wpi.first.units.Units.*;

import java.io.IOException;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.json.simple.parser.ParseException;
import org.team2168.Constants.CameraConstants;
import org.team2168.subsystems.SwerveDrivetrain.TunerConstants.TunerSwerveDrivetrain;
import org.team2168.utils.LimelightHelpers;
import org.team2168.utils.LimelightHelpers.LimelightResults;
import org.team2168.utils.LimelightHelpers.PoseEstimate;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.fasterxml.jackson.core.StreamReadFeature;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class Swerve extends TunerSwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    private SlewRateLimiter xLimiter = new SlewRateLimiter(10.0);
    private SlewRateLimiter yLimiter = new SlewRateLimiter(10.0);
    private SlewRateLimiter rotLimiter = new SlewRateLimiter(Units.degreesToRadians(1350));
    private SlewRateLimiter xSlowLimiter = new SlewRateLimiter(20);
    private SlewRateLimiter ySlowLimiter = new SlewRateLimiter(20);
    private SlewRateLimiter slowRotLimiter = new SlewRateLimiter(Units.degreesToRadians(5000));

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    private final SwerveRequest.ApplyRobotSpeeds robotSpeeds = new SwerveRequest.ApplyRobotSpeeds();
    private final PPHolonomicDriveController controller = new PPHolonomicDriveController(new PIDConstants(4, 0, 0),
            new PIDConstants(4, 0, 0));

    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();
    private final SwerveRequest.FieldCentric fieldCentricDrive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.Velocity);
    /*
     * SysId routine for characterizing translation. This is used to find PID gains
     * for the drive motors.
     */
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null, // Use default ramp rate (1 V/s)
                    Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    output -> setControl(m_translationCharacterization.withVolts(output)),
                    null,
                    this));

    /*
     * SysId routine for characterizing steer. This is used to find PID gains for
     * the steer motors.
     */
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null, // Use default ramp rate (1 V/s)
                    Volts.of(7), // Use dynamic voltage of 7 V
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdSteer_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    volts -> setControl(m_steerCharacterization.withVolts(volts)),
                    null,
                    this));

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle
     * HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on
     * importing the log to SysId.
     */
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    /* This is in radians per second², but SysId only supports "volts per second" */
                    Volts.of(Math.PI / 6).per(Second),
                    /* This is in radians per second, but SysId only supports "volts" */
                    Volts.of(Math.PI),
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdRotation_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    output -> {
                        /* output is actually radians per second, but SysId only supports "volts" */
                        setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                        /* also log the requested output for SysId */
                        SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
                    },
                    null,
                    this));

    /* The SysId routine to test */
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not
     * construct
     * the devices themselves. If they need the devices, they can access them
     * through
     * getters in the classes.
     *
     * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
     * @param modules             Constants for each specific module
     */
    public Swerve(
            SwerveDrivetrainConstants drivetrainConstants,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }

        try { // robot weight is 103.6lbs
            AutoBuilder.configure(() -> this.getState().Pose, this::resetPose, () -> this.getState().Speeds,
                    (chassisSpeeds, feedforward) -> setControl(robotSpeeds.withSpeeds(chassisSpeeds)
                            .withWheelForceFeedforwardsX(feedforward.robotRelativeForcesX())
                            .withWheelForceFeedforwardsY(feedforward.robotRelativeForcesY())),
                    controller, RobotConfig.fromGUISettings(), () -> DriverStation.getAlliance().get() == Alliance.Red,
                    this);
        } catch (IOException | ParseException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
        setStateStdDevs(VecBuilder.fill(0.1,0.1,10000));
        LimelightHelpers.setCameraPose_RobotSpace("limelight-frontes", CameraConstants.FRONT_HIGH_FORWARD_OFFSET,
                CameraConstants.FRONT_HIGH_STRAFE_OFFSET,
                CameraConstants.FRONT_HIGH_VERTICAL_OFFSET, CameraConstants.FRONT_HIGH_ROLL,
                CameraConstants.FRONT_HIGH_PITCH, CameraConstants.FRONT_HIGH_YAW);
        LimelightHelpers.setCameraPose_RobotSpace("limelight-central", CameraConstants.FRONT_LOW_FORWARD_OFFSET,
                CameraConstants.FRONT_LOW_STRAFE_OFFSET,
                CameraConstants.FRONT_LOW_VERTICAL_OFFSET, CameraConstants.FRONT_LOW_ROLL,
                CameraConstants.FRONT_LOW_PITCH, CameraConstants.FRONT_LOW_YAW);
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not
     * construct
     * the devices themselves. If they need the devices, they can access them
     * through
     * getters in the classes.
     *
     * @param drivetrainConstants     Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency The frequency to run the odometry loop. If
     *                                unspecified or set to 0 Hz, this is 250 Hz on
     *                                CAN FD, and 100 Hz on CAN 2.0.
     * @param modules                 Constants for each specific module
     */
    public Swerve(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not
     * construct
     * the devices themselves. If they need the devices, they can access them
     * through
     * getters in the classes.
     *
     * @param drivetrainConstants       Drivetrain-wide constants for the swerve
     *                                  drive
     * @param odometryUpdateFrequency   The frequency to run the odometry loop. If
     *                                  unspecified or set to 0 Hz, this is 250 Hz
     *                                  on
     *                                  CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation The standard deviation for odometry
     *                                  calculation
     *                                  in the form [x, y, theta]ᵀ, with units in
     *                                  meters
     *                                  and radians
     * @param visionStandardDeviation   The standard deviation for vision
     *                                  calculation
     *                                  in the form [x, y, theta]ᵀ, with units in
     *                                  meters
     *                                  and radians
     * @param modules                   Constants for each specific module
     */
    public Swerve(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            Matrix<N3, N1> odometryStandardDeviation,
            Matrix<N3, N1> visionStandardDeviation,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation,
                modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    /**
     * Returns a command that applies the specified control request to this swerve
     * drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the
     * odometry pose estimate
     * while still accounting for measurement noise.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision
     *                              camera.
     * @param timestampSeconds      The timestamp of the vision measurement in
     *                              seconds.
     */
    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the
     * odometry pose estimate
     * while still accounting for measurement noise.
     * <p>
     * Note that the vision measurement standard deviations passed into this method
     * will continue to apply to future measurements until a subsequent call to
     * {@link #setVisionMeasurementStdDevs(Matrix)} or this method.
     *
     * @param visionRobotPoseMeters    The pose of the robot as measured by the
     *                                 vision camera.
     * @param timestampSeconds         The timestamp of the vision measurement in
     *                                 seconds.
     * @param visionMeasurementStdDevs Standard deviations of the vision pose
     *                                 measurement
     *                                 in the form [x, y, theta]ᵀ, with units in
     *                                 meters and radians.
     */
    @Override
    public void addVisionMeasurement(
            Pose2d visionRobotPoseMeters,
            double timestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds),
                visionMeasurementStdDevs);
    }

    public Command runDriveWithJoystick(CommandXboxController joystick, double maxSpeed, double maxAngularSpeed) {

        return applyRequest(() -> fieldCentricDrive.withVelocityX(xLimiter.calculate(-joystick.getLeftY() * maxSpeed))
                .withVelocityY(yLimiter.calculate(-joystick.getLeftX() * maxSpeed))
                .withRotationalRate(rotLimiter.calculate(-joystick.getRightX() * maxAngularSpeed)));

    }

    public Command drivePath(String pathName) {
        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
            // PathConstraints constraints = new PathConstraints()
            return AutoBuilder.followPath(path);
        } catch (FileVersionException | IOException | ParseException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
            return Commands.none(); // or return a default Command
        }
    }

    public Command runPathPlannerCommand(String fileName) {
        return new PathPlannerAuto(fileName);
    }

    public Command pathFind(Pose2d targetPose) {
        PathConstraints pathConstraints = new PathConstraints(MetersPerSecond.of(3.5), MetersPerSecondPerSecond.of(1.5),
                RadiansPerSecond.of(2 * Math.PI), RadiansPerSecondPerSecond.of(2 * Math.PI));
        return AutoBuilder.pathfindToPose(targetPose, pathConstraints);
    }

    public Pose2d getStartingPosePathplanner(String filename) {
        try {
            return PathPlannerPath.fromPathFile(filename).getStartingHolonomicPose().orElse(new Pose2d());
        } catch (FileVersionException | IOException | ParseException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
        return new Pose2d(); // Return a default Pose2d in case of an exception
    }

    public Command resetPosePathplanner(String pathname) {
        try {
            return AutoBuilder
                    .resetOdom(PathPlannerPath.fromPathFile(pathname).getStartingHolonomicPose().orElse(new Pose2d()));
        } catch (FileVersionException | IOException | ParseException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
            return Commands.none(); // or return a default Command
        }
    }

    public void changeAutoSpeedFromClimberHeight(DoubleSupplier climbHeightRotations) {

    }

    private void processPoseEstimate(PoseEstimate poseEstimate) {
        if (LimelightHelpers.validPoseEstimate(poseEstimate) && poseEstimate.rawFiducials[0].ambiguity < 0.2
                && Math.abs(getPigeon2().getAngularVelocityZWorld().getValueAsDouble()) <= 285) {
            Pose2d visionEstimate = poseEstimate.pose;
            addVisionMeasurement(visionEstimate, poseEstimate.timestampSeconds);
            // scaleStdDevs(poseEstimate, 0.5);
        }
    }

    private void scaleStdDevs(PoseEstimate estimate, double scaleFactor) {
        double tagDistance = estimate.avgTagDist;
        double xStdScaled = 0.001 * Math.pow(tagDistance / scaleFactor, 2);
        double yStdScaled = 0.001 * Math.pow(tagDistance / scaleFactor, 2);
        double rotScaled = 100000;
        setStateStdDevs(VecBuilder.fill(xStdScaled, yStdScaled, rotScaled));
    }

    @Override
    public void periodic() {
        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply
         * it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts
         * mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is
         * disabled.
         * This ensures driving behavior doesn't change until an explicit disable event
         * occurs during testing.
         */
        setStateStdDevs(VecBuilder.fill(0.1,0.1,99999999));
        double yaw = this.getPigeon2().getYaw().getValueAsDouble();
        LimelightHelpers.SetRobotOrientation("limelight-frontes", yaw, 0.0, 0.0, 0.0, 0.0, 0.0);
        LimelightHelpers.SetRobotOrientation("limelight-central", yaw, 0.0, 0.0, 0.0, 0.0, 0.0);

        PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-frontes");
        PoseEstimate poseEstimateTwo = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-central");
        processPoseEstimate(poseEstimate);
        processPoseEstimate(poseEstimateTwo);

        // SmartDashboard.putNumber("cancoder 0", getModule(0).getEncoder().getAbsolutePosition().getValueAsDouble());
        // SmartDashboard.putNumber("cancoder 1", getModule(1).getEncoder().getAbsolutePosition().getValueAsDouble());
        // SmartDashboard.putNumber("cancoder 2", getModule(2).getEncoder().getAbsolutePosition().getValueAsDouble());
        // SmartDashboard.putNumber("cancoder 3", getModule(3).getEncoder().getAbsolutePosition().getValueAsDouble());
        // SmartDashboard.putNumber("angularZSpeed", getPigeon2().getAngularVelocityZWorld().getValueAsDouble());
        // SmartDashboard.putNumber("Gyro Yaw", getPigeon2().getYaw().getValueAsDouble());

        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                        allianceColor == Alliance.Red
                                ? kRedAlliancePerspectiveRotation
                                : kBlueAlliancePerspectiveRotation);
                m_hasAppliedOperatorPerspective = true;
            });
        }
    }
}
