// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.CoralManipulator;

import org.team2168.subsystems.CoralFlywheel;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveFlywheelWithJoystick extends Command {
  /** Creates a new DriveFlywheelWithJoystick. */
  private CoralFlywheel coralFlywheel;
  private CommandXboxController controllerY;

  public DriveFlywheelWithJoystick(CoralFlywheel coralFlywheel, CommandXboxController controllerY) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.coralFlywheel = coralFlywheel;
    this.controllerY = controllerY;

    addRequirements(coralFlywheel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (controllerY.getRightY() < 0.5 && controllerY.getRightY() > -0.5) {
      coralFlywheel.setFlywheelSpeed(0.0);
    }
    else if (controllerY.axisGreaterThan(5, 0.5).getAsBoolean()) {
      coralFlywheel.setFlywheelSpeed(-0.15);
    }
    else if (controllerY.axisLessThan(5, -0.5).getAsBoolean()) {
      coralFlywheel.setFlywheelSpeed(0.15);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    coralFlywheel.setFlywheelSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
