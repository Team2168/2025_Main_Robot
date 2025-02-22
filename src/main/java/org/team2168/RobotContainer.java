// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168;

import org.team2168.Constants.OperatorConstants;
import org.team2168.commands.Autos;
import org.team2168.commands.CloseClimber;
import org.team2168.commands.DriveClimber;
import org.team2168.commands.ExampleCommand;
import org.team2168.commands.lift.DriveLift;
import org.team2168.commands.lift.DriveLiftHeights;
import org.team2168.subsystems.Climber;
import org.team2168.subsystems.ExampleSubsystem;
import org.team2168.subsystems.Lift;
import org.team2168.subsystems.Lift.LiftHeights;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import io.github.oblarg.oblog.Logger;

import org.team2168.Constants.ClimberConstants;
import org.team2168.Constants.Controllers;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private final Climber climber = new Climber(); 
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final Lift m_Lift = new Lift();

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
    testJoystick.rightStick().whileTrue(new DriveLift(m_Lift, () -> testJoystick.getRightY()));
    operatorJoystick.a().onTrue(new DriveLiftHeights(m_Lift, LiftHeights.BARGE));
    operatorJoystick.b().onTrue(new DriveLiftHeights(m_Lift, LiftHeights.L2));
    operatorJoystick.y().onTrue(new DriveLiftHeights(m_Lift, LiftHeights.L3));
    operatorJoystick.x().onTrue(new DriveLiftHeights(m_Lift, LiftHeights.L4));
    operatorJoystick.povLeft().whileTrue(new CloseClimber(climber));
    operatorJoystick.povRight().whileTrue(new DriveClimber(climber, ()-> ClimberConstants.openingSpeed));
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
