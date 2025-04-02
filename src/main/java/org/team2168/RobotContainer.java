// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.team2168.Constants.Controllers;
import org.team2168.Constants.OperatorConstants;
import org.team2168.Constants.ClimberConstants;

import org.team2168.commands.Autos;
import org.team2168.commands.ExampleCommand;
import org.team2168.commands.Climber.CloseClimber;
import org.team2168.commands.Climber.DriveClimber;
import org.team2168.commands.CoralManipulator.BumpCoralPivotAngleDown;
import org.team2168.commands.CoralManipulator.BumpCoralPivotAngleUp;
import org.team2168.commands.CoralManipulator.DriveCoralFlywheel;
import org.team2168.commands.CoralManipulator.DriveFlywheelUntilCoral;
import org.team2168.commands.CoralManipulator.DriveFlywheelUntilNoCoral;
import org.team2168.commands.CoralManipulator.SetCoralPivotAngle;
// import org.team2168.commands.Drive.DriveToPose;
import org.team2168.commands.CoralManipulator.DriveFlywheelWithJoystick;
import org.team2168.commands.lift.DriveLiftHeights;
import org.team2168.commands.LED.LEDStatus;

import org.team2168.subsystems.CoralFlywheel;
import org.team2168.subsystems.CoralPivot;
import org.team2168.subsystems.CoralPivot.CORAL_PIVOT_POSITION;
import org.team2168.subsystems.Climber;
import org.team2168.subsystems.Lift;
import org.team2168.subsystems.Lift.LiftHeights;
import org.team2168.subsystems.CageDetector;
import org.team2168.subsystems.LEDs;

import org.team2168.commands.IntakePivot.setIntakePivotAngleHigher;
import org.team2168.commands.IntakePivot.setIntakePivotAngleLower;
import org.team2168.commands.IntakePivot.setIntakePivotPosition;
import org.team2168.commands.IntakeWheel.setIntakeSpeed;

import org.team2168.subsystems.ExampleSubsystem;
import org.team2168.subsystems.SwerveDrivetrain.Swerve;
import org.team2168.subsystems.SwerveDrivetrain.TunerConstants;
// import org.team2168.utils.PosesUtil;
import org.team2168.utils.Telemetry;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.LTVDifferentialDriveController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import io.github.oblarg.oblog.Logger;

import org.team2168.subsystems.algaeIntakeWheel;
import org.team2168.subsystems.algaeIntakePivot;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
        // The robot's subsystems and commands are defined here...
        SendableChooser<Command> autoChooser = new SendableChooser<>();
        private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
        private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
        private final Swerve swerve = TunerConstants.createDrivetrain();
        private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem(); // Use open-loop control for drive
                                                                                    // motors

        private final Telemetry logger = new Telemetry(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond), swerve);

        private final CoralFlywheel coralFlywheel = new CoralFlywheel();
        private final CoralPivot coralPivot = new CoralPivot();

        private final Lift lift = new Lift();

        private final Climber climber = new Climber();

        private final LEDs leds = new LEDs();
        private final CageDetector cageDetector = new CageDetector();

        private final algaeIntakePivot algaeintakePivot = new algaeIntakePivot();
        private final algaeIntakeWheel algaeintakeWheel = new algaeIntakeWheel();

        // Replace with CommandPS4Controller or CommandJoystick if needed
        public CommandXboxController driverJoystick = new CommandXboxController(Controllers.DRIVER_JOYSTICK);
        public CommandXboxController operatorJoystick = new CommandXboxController(Controllers.OPERATOR_JOYSTICK);
        public CommandXboxController testJoystick = new CommandXboxController(Controllers.TEST_JOYSTICK);
        public Autos autos = new Autos(swerve, coralFlywheel, coralPivot, lift);

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                // Configure the trigger bindings
                configureAutonomous();
                configureBindings();
                Logger.configureLoggingAndConfig(this, false);
        }

        /**
         * Use this method to define your trigger->command mappings. Triggers can be
         * created via the
         * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
         * an arbitrary
         * predicate, or via the named factories in {@link
         * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
         * {@link
         * CommandXboxController
         * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
         * PS4} controllers or
         * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
         * joysticks}.
         */
        // bindings for moving the climber teehee
        private void configureBindings() {
                // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
                // new Trigger(m_exampleSubsystem::exampleCondition)
                // .onTrue(new ExampleCommand(m_exampleSubsystem));

                // Schedule `exampleMethodCommand` when the Xbox controller's B button is
                // pressed,
                // cancelling on release.
                // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());

                // driverJoystick.leftBumper().onTrue(
                // new DriveToPose(() -> swerve.getState().Pose,
                // swerve, () -> swerve.getState().Speeds));

                // driverJoystick.rightBumper().onTrue(
                // new DriveToPose(() -> swerve.getState().Pose,
                // swerve, () -> swerve.getState().Speeds));

                swerve.setDefaultCommand(
                                swerve.runDriveWithJoystick(driverJoystick, MaxSpeed, MaxAngularRate, false));

                driverJoystick.leftTrigger()
                                .whileTrue(
                                                swerve.runDriveWithJoystick(driverJoystick,
                                                                MaxSpeed / Constants.DrivePIDConstants.SLOW_FACTOR,
                                                                MaxAngularRate / Constants.DrivePIDConstants.SLOW_FACTOR, 
                                                                false));
                driverJoystick.rightTrigger()
                                .whileTrue(
                                                swerve.runDriveWithJoystick(driverJoystick,
                                                                MaxSpeed/Constants.DrivePIDConstants.VERY_SLOW_FACTOR,
                                                                MaxAngularRate/Constants.DrivePIDConstants.VERY_SLOW_FACTOR,
                                                                true));

                driverJoystick.back().onTrue(swerve.applyRequest(
                                () -> new SwerveRequest.PointWheelsAt()
                                                .withModuleDirection(Rotation2d.fromDegrees(180))));

                driverJoystick.y()
                                .whileTrue(new DriveClimber(climber, ClimberConstants.CLOSING_SPEED));

                swerve.registerTelemetry(logger::telemeterize);

                leds.setDefaultCommand(new LEDStatus(leds, cageDetector, coralFlywheel));

                /*
                 * control coral in intake buttons: (needs testing) sets the coral intake to a
                 * very slow speed depending on if the operator moves the right stick up or down
                 */
                coralFlywheel.setDefaultCommand(new DriveFlywheelWithJoystick(coralFlywheel, operatorJoystick));

                /*
                 * elevator and coral pivot reset button: sets elevator and coral pivot position
                 * to 0
                 */
                operatorJoystick.rightTrigger()
                                .onTrue(Commands.parallel(
                                                new SetCoralPivotAngle(coralPivot,
                                                                CORAL_PIVOT_POSITION.ZERO.getPivotPositon()),
                                                new DriveLiftHeights(lift, LiftHeights.ZERO.getValue())));

                /*
                 * coral intake button: sets elevator and coral pivot to the position to intake
                 * coral while also running the coral flywheel until line break sensor senses
                 * coral
                 */
                operatorJoystick.rightBumper()
                                .onTrue((Commands.parallel(
                                                new SetCoralPivotAngle(coralPivot,
                                                                CORAL_PIVOT_POSITION.INTAKE.getPivotPositon()),
                                                new DriveLiftHeights(lift, LiftHeights.INTAKE.getValue()))))
                                .whileTrue(new DriveFlywheelUntilCoral(coralFlywheel, -0.4));

                /* L1-L4 buttons: sets elevator and coral pivot to desired reef branch */
                operatorJoystick.a()
                                .onTrue(Commands.parallel(
                                                new SetCoralPivotAngle(coralPivot,
                                                                CORAL_PIVOT_POSITION.BARGE.getPivotPositon()),
                                                new DriveLiftHeights(lift, LiftHeights.BARGE.getValue())));
                operatorJoystick.b()
                                .onTrue(Commands.parallel(
                                                new SetCoralPivotAngle(coralPivot,
                                                                CORAL_PIVOT_POSITION.L2.getPivotPositon()),
                                                new DriveLiftHeights(lift, LiftHeights.L2.getValue())));
                operatorJoystick.y()
                                .onTrue(Commands.parallel(
                                                new SetCoralPivotAngle(coralPivot,
                                                                CORAL_PIVOT_POSITION.L3.getPivotPositon()),
                                                new DriveLiftHeights(lift, LiftHeights.L3.getValue())));
                operatorJoystick.x()
                                .onTrue(Commands.parallel(
                                                new SetCoralPivotAngle(coralPivot,
                                                                CORAL_PIVOT_POSITION.L4.getPivotPositon()),
                                                new DriveLiftHeights(lift, LiftHeights.L4.getValue())));

                /*
                 * taking algae off upper reef: when clicked, it'll move the coral pivot to a
                 * spot to take algae off, and when held it will drive the wheels as well
                 */
                operatorJoystick.povUp()
                                .onTrue(new SetCoralPivotAngle(coralPivot,
                                                CORAL_PIVOT_POSITION.UPPER_ALGAE.getPivotPositon()))
                                .whileTrue(new DriveCoralFlywheel(coralFlywheel, -0.5));

                /*
                 * taking algae off lower reef: when clicked, it'll move the coral pivot to a
                 * spot to take algae off, and when held it will drive the wheels as well
                 */
                operatorJoystick.povDown()
                                .onTrue(new SetCoralPivotAngle(coralPivot,
                                                CORAL_PIVOT_POSITION.LOWER_ALGAE.getPivotPositon()))
                                .whileTrue(new DriveCoralFlywheel(coralFlywheel, 0.5));

                /* intake algae button */
                operatorJoystick.leftBumper()
                                .onTrue(new setIntakePivotPosition(algaeintakePivot, -5.238)) // originally: -4.538
                                .whileTrue(new setIntakeSpeed(algaeintakeWheel, 0.75));

                /* shoot algae button */
                operatorJoystick.leftTrigger().whileTrue(new setIntakeSpeed(algaeintakeWheel, -1.0));

                /* algae reset position button */
                operatorJoystick.povRight()
                                .onTrue(new setIntakePivotPosition(algaeintakePivot, -0.5));

                /* sets to intake cage */
                operatorJoystick.povLeft()
                                .onTrue(new setIntakePivotPosition(algaeintakePivot, -7.5))
                                .whileTrue(new setIntakeSpeed(algaeintakeWheel, -0.5));

                // testJoystick.povRight().whileTrue(new DriveClimber(climber, () ->
                // ClimberConstants.OPENING_SPEED));

                testJoystick.rightBumper().whileTrue(new DriveFlywheelUntilCoral(coralFlywheel, -0.6));
                testJoystick.leftBumper().whileTrue(new DriveFlywheelUntilNoCoral(coralFlywheel, 0.6));

                testJoystick.rightTrigger().onTrue(new BumpCoralPivotAngleUp(coralPivot)); // test
                testJoystick.leftTrigger().onTrue(new BumpCoralPivotAngleDown(coralPivot));

        }

        private void configureAutonomous() {
                autoChooser.addOption("MiddleLeave", autos.middleLeave());
                autoChooser.addOption("MiddleScore", autos.middleScore());
                // autoChooser.addOption("LeftLeaveScoreTwoCoral", autos.leftTwoPiece());
                // autoChooser.addOption("RightLeaveScoreTwoCoral", autos.rightTwoPiece());
                autoChooser.addOption("RightLeave", autos.rightLeave());
                autoChooser.addOption("LeftLeave", autos.leftLeave());
                autoChooser.addOption("L1LeftScore", autos.leftScoreSingle());
                autoChooser.addOption("L1RightScore", autos.rightScoreSingle());
                autoChooser.addOption("L4RightScore", autos.rightScoreSingleL4());
                autoChooser.addOption("L4LeftScore", autos.leftScoreSingleL4());
                autoChooser.addOption("L1LeftTwoPiece", autos.leftTwoPieceL1());
                autoChooser.addOption("L1RightTwoPiece", autos.rightTwoPieceL1());
                autoChooser.addOption("middleScoreL4", autos.middleL4());
                autoChooser.addOption("rightTwoPieceL4", autos.rightTwoPieceL4());
                SmartDashboard.putData("Auto Mode", autoChooser);

        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                // An example command will be run in autonomous
                Command auto = autoChooser.getSelected();
                if (auto != null) {
                        return auto;
                } else {
                        return Commands.none();
                }
        }

}
