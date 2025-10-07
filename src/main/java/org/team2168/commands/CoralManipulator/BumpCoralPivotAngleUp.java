// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package org.team2168.commands.CoralManipulator;

import org.team2168.subsystems.CoralPivot;

import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class BumpCoralPivotAngleUp extends Command {
  /** Creates a new setIntakePivotPosition. */

  private CoralPivot coralPivot;

  public BumpCoralPivotAngleUp(CoralPivot coralPivot) {
    this.coralPivot = coralPivot;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(coralPivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    coralPivot.setCoralPivotPosition(coralPivot.getCoralPivotPositionRot() + 1.0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
