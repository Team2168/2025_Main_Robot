// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.IntakePivot;

import org.team2168.subsystems.algaeIntakePivot;

import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class setIntakePivotAngleLower extends Command {

  private algaeIntakePivot iPivot;

  public setIntakePivotAngleLower(algaeIntakePivot iPivot) {
    this.iPivot = iPivot;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(iPivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    iPivot.setIntakePivotPosition(iPivot.getIntakePivotAngle() - 1.0);
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
