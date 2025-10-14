// package org.team2168.commands.Drive;

// import java.util.function.Supplier;

// import org.team2168.Constants;
// import org.team2168.Constants.PoseConstants;
// import org.team2168.subsystems.SwerveDrivetrain.Swerve;
// import org.team2168.utils.PosesUtil;

// import com.ctre.phoenix6.swerve.SwerveRequest;

// import edu.wpi.first.math.controller.HolonomicDriveController;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Transform2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.math.trajectory.Trajectory;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;

// /* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
// public class DriveToPose extends Command {
//   Supplier<Pose2d> targetPose;
//   Supplier<Pose2d> robotPose;
//   Supplier<ChassisSpeeds> currentSpeeds;
//   ProfiledPIDController xController;// Tune
//   ProfiledPIDController yController;// Tune
//   ProfiledPIDController thetaController; // Tune
//   SwerveRequest.ApplyRobotSpeeds robotSpeeds = new SwerveRequest.ApplyRobotSpeeds();
//   Swerve swerve;
//   boolean finished = false;

//   public DriveToPose(Supplier<Pose2d> robotPose, Swerve swerve,
//       Supplier<ChassisSpeeds> currentSpeeds) {
//     this.swerve = swerve;
//     this.robotPose = robotPose;
//     this.currentSpeeds = currentSpeeds;

//     xController = new ProfiledPIDController(0.8, 0, 0,
//         new TrapezoidProfile.Constraints(Constants.DrivePIDConstants.xMaxVelocity,
//             Constants.DrivePIDConstants.yMaxAcceleration));
//     yController = new ProfiledPIDController(0.8, 0, 0,
//         new TrapezoidProfile.Constraints(Constants.DrivePIDConstants.yMaxVelocity,
//             Constants.DrivePIDConstants.yMaxAcceleration));
//     thetaController = new ProfiledPIDController(4, 0.0, 0.02, new TrapezoidProfile.Constraints(
//         Constants.DrivePIDConstants.thetaMaxVelocity, Constants.DrivePIDConstants.thetaMaxAcceleration));
//     thetaController.enableContinuousInput(-Math.PI, Math.PI);

//     // Set tolerances for the controllers
//     xController.setTolerance(0.01);
//     yController.setTolerance(0.01);
//     thetaController.setTolerance(Units.degreesToRadians(1));

//     addRequirements(swerve);
//   }

//   @Override
//   public void initialize() {
//     targetPose = () -> PosesUtil.transformPoseDirection(true, PosesUtil.findNearestScoringPose(swerve.getState().Pose,
//     PosesUtil.getScorePositionsFromAlliance(DriverStation.getAlliance().get())));
//     Pose2d currentPose = robotPose.get();
//     ChassisSpeeds currentRobotSpeeds = currentSpeeds.get();
//     xController.reset(new TrapezoidProfile.State(currentPose.getX(), currentRobotSpeeds.vxMetersPerSecond));
//     yController.reset(new TrapezoidProfile.State(currentPose.getY(), currentRobotSpeeds.vyMetersPerSecond));
//     thetaController.reset(
//         new TrapezoidProfile.State(currentPose.getRotation().getRadians(), currentRobotSpeeds.omegaRadiansPerSecond));
//   }

//   @Override
//   public void execute() {

//     // System.out.println((xController.atGoal() && yController.atGoal() && thetaController.atGoal()));
//     Pose2d goalPose = targetPose.get();
//     Pose2d currentPose = robotPose.get();

//     double xSetpoint = xController.calculate(currentPose.getX(),
//         goalPose.getX());
//     double ySetpoint = yController.calculate(currentPose.getY(),
//         goalPose.getY());
//     double thetaSetpoint = thetaController.calculate(
//         currentPose.getRotation().getRadians(),
//         goalPose.getRotation().getRadians());

//     double xSpeed = xSetpoint;
//     double ySpeed = ySetpoint;
//     double omegaSpeed = thetaSetpoint;
//     swerve.setControl(robotSpeeds.withSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(
//         xSpeed, ySpeed,
//         omegaSpeed, currentPose.getRotation())));
    
     
//   }

//   @Override
//   public void end(boolean interrupted) {
//     swerve.setControl(robotSpeeds.withSpeeds(new ChassisSpeeds(0, 0, 0)));
//   }

//   @Override
//   public boolean isFinished() {
//    return xController.atGoal() && yController.atGoal() && thetaController.atGoal();
//   }
// }