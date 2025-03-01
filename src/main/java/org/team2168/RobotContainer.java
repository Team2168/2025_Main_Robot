// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168;

import org.team2168.Constants.Controllers;
import org.team2168.Constants.OperatorConstants;
import org.team2168.commands.Autos;
import org.team2168.commands.ExampleCommand;

import org.team2168.commands.CloseClimber;
import org.team2168.commands.DriveClimber;
import org.team2168.commands.DriveLiftTest;
import org.team2168.commands.BumpCoralPivotAngleDown;
import org.team2168.commands.BumpCoralPivotAngleUp;
import org.team2168.commands.DriveFlywheelUntilCoral;
import org.team2168.commands.DriveFlywheelUntilNoCoral;
import org.team2168.commands.SetCoralPivotAngle;
import org.team2168.commands.lift.DriveLift;
import org.team2168.commands.lift.DriveLiftHeights;
import org.team2168.commands.LEDStatus;

import org.team2168.subsystems.CoralFlywheel;
import org.team2168.subsystems.CoralPivot;
import org.team2168.subsystems.CoralPivot.CORAL_PIVOT_POSITION;
import org.team2168.subsystems.Climber;
import org.team2168.subsystems.ExampleSubsystem;
import org.team2168.subsystems.Lift;
import org.team2168.subsystems.Lift.LiftHeights;
import org.team2168.subsystems.CageDetector;
import org.team2168.subsystems.LEDs;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.math.controller.LTVDifferentialDriveController;
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
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  
  private final CoralFlywheel coralflyWheel = new CoralFlywheel();
  private final CoralPivot coralPivot = new CoralPivot();
      
  private final Lift m_Lift = new Lift();
  
  private final Climber climber = new Climber(); 

  private final LEDs leds = new LEDs();
  private final CageDetector cageDetector = new CageDetector();

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
    // new Trigger(m_exampleSubsystem::exampleCondition)
    //     .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.

    m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());

    leds.setDefaultCommand(new LEDStatus(leds, cageDetector));
    

    testJoystick.rightBumper().whileTrue(new DriveFlywheelUntilCoral(coralflyWheel, -0.4)); 
    testJoystick.leftBumper().whileTrue(new DriveFlywheelUntilNoCoral(coralflyWheel, 0.4));

    testJoystick.rightTrigger().onTrue(new BumpCoralPivotAngleUp(coralPivot)); //brings pivot back to 0
    testJoystick.leftTrigger().onTrue(new BumpCoralPivotAngleDown(coralPivot));

    testJoystick.a().onTrue(new SetCoralPivotAngle(coralPivot, 0.0));
    testJoystick.b().onTrue(new SetCoralPivotAngle(coralPivot, 5.0));
    testJoystick.y().onTrue(new SetCoralPivotAngle(coralPivot, 10.0));
    testJoystick.x().onTrue(new SetCoralPivotAngle(coralPivot, 16.0));
    
    testJoystick.rightStick().whileTrue(new DriveLift(m_Lift, () -> testJoystick.getRightY()));
  

    operatorJoystick.rightTrigger().whileTrue(new DriveFlywheelUntilNoCoral(coralflyWheel, 0.4));
    operatorJoystick.rightBumper().whileTrue(new DriveFlywheelUntilCoral(coralflyWheel, -0.4));

    operatorJoystick.a().onTrue(new SetCoralPivotAngle(coralPivot, CORAL_PIVOT_POSITION.BARGE.getPivotPositon()));
    operatorJoystick.b().onTrue(new SetCoralPivotAngle(coralPivot, CORAL_PIVOT_POSITION.L2.getPivotPositon()));
    operatorJoystick.y().onTrue(new SetCoralPivotAngle(coralPivot, CORAL_PIVOT_POSITION.L3.getPivotPositon()));
    operatorJoystick.x().onTrue(new SetCoralPivotAngle(coralPivot, CORAL_PIVOT_POSITION.L4.getPivotPositon()));

    operatorJoystick.a().onTrue(new DriveLiftTest(m_Lift, LiftHeights.BARGE.getValue()));
    operatorJoystick.b().onTrue(new DriveLiftTest(m_Lift, LiftHeights.L2.getValue()));
    operatorJoystick.y().onTrue(new DriveLiftTest(m_Lift, LiftHeights.L3.getValue()));
    operatorJoystick.x().onTrue(new DriveLiftTest(m_Lift, LiftHeights.L4.getValue()));
    
    operatorJoystick.povLeft().whileTrue(new CloseClimber(climber));
    operatorJoystick.povRight().whileTrue(new DriveClimber(climber, ()-> ClimberConstants.openingSpeed));

    // testJoystick.a().onTrue(new DriveLiftTest(m_Lift, LiftHeights.BARGE.getValue()));
    // testJoystick.b().onTrue(new DriveLiftTest(m_Lift, LiftHeights.L2.getValue()));
    // testJoystick.y().onTrue(new DriveLiftTest(m_Lift, LiftHeights.L3.getValue()));
    // testJoystick.x().onTrue(new DriveLiftTest(m_Lift, LiftHeights.L4.getValue()));
    
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
