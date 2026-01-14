// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.team2168.Constants.Controllers;
import org.team2168.Constants.ClimberConstants;


// import org.team2168.commands.Drive.DriveToPose;



import org.team2168.subsystems.ExampleSubsystem;
import org.team2168.subsystems.SwerveDrivetrain.Swerve;
import org.team2168.subsystems.SwerveDrivetrain.TunerConstants;
// import org.team2168.utils.PosesUtil;
import org.team2168.utils.Telemetry;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import io.github.oblarg.oblog.Logger;


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


        // Replace with CommandPS4Controller or CommandJoystick if needed
        public CommandXboxController driverJoystick = new CommandXboxController(Controllers.DRIVER_JOYSTICK);
        public CommandXboxController operatorJoystick = new CommandXboxController(Controllers.OPERATOR_JOYSTICK);
        public CommandXboxController testJoystick = new CommandXboxController(Controllers.TEST_JOYSTICK);

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                // Configure the trigger bindings
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


                swerve.registerTelemetry(logger::telemeterize);

             
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
