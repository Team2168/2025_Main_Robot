// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168;

import org.team2168.Constants.Controllers;
import org.team2168.commands.Autos;
import org.team2168.commands.ExampleCommand;
import org.team2168.commands.IntakePivot.setIntakePivotAngleHigher;
import org.team2168.commands.IntakePivot.setIntakePivotAngleLower;
import org.team2168.commands.IntakePivot.setIntakePivotPosition;
import org.team2168.commands.IntakeWheel.setIntakeSpeed;

import org.team2168.subsystems.ExampleSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import io.github.oblarg.oblog.Logger;

import org.team2168.subsystems.algaeIntakeWheel;
import org.team2168.subsystems.algaeIntakePivot;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private final algaeIntakePivot algaeintakePivot = new algaeIntakePivot();
  private final algaeIntakeWheel algaeintakeWheel = new algaeIntakeWheel();

  

  // Replace with CommandPS4Controller or CommandJoystick if needed
  public CommandXboxController driverJoystick = new CommandXboxController(Controllers.DRIVER_JOYSTICK);
  public CommandXboxController operatorJoystick = new CommandXboxController(Controllers.OPERATOR_JOYSTICK);
  public CommandXboxController testJoystick = new CommandXboxController(Controllers.TEST_JOYSTICK);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    Logger.configureLoggingAndConfig(this, false);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  //bindings for moving the climber teehee
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    //m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());

    testJoystick.leftBumper().whileTrue(new setIntakeSpeed(algaeintakeWheel, 0.5)); //intake
    testJoystick.pov(0).whileTrue(new setIntakePivotAngleHigher(algaeintakePivot)); //take off upper (up dpad)
    testJoystick.pov(180).whileTrue(new setIntakePivotAngleLower(algaeintakePivot)); //take off lower (down dpad)

    /* intake algae button */
    operatorJoystick.leftBumper()
        .onTrue(new setIntakePivotPosition(algaeintakePivot, -8.0))
            .whileTrue(new setIntakeSpeed(algaeintakeWheel, 0.5));
    
    /* shoot algae button */
    operatorJoystick.leftTrigger().whileTrue(new setIntakeSpeed(algaeintakeWheel, -0.5));

    /* algae reset position button */
    operatorJoystick.povRight()
        .onTrue(new setIntakePivotPosition(algaeintakePivot, 0.0));
  

    // new Trigger(m_exampleSubsystem::exampleCondition)
    //     .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
  
}
