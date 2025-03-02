// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.CoralManipulator;

import org.team2168.subsystems.CoralPivot;

import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetCoralPivotAngle extends Command {
  private CoralPivot coralPivot;
  private double coralPivPosition;
  private double tolerance = 0.5;

  /** Creates a new SetCoralPivotAngle. */
  public SetCoralPivotAngle(CoralPivot coralPivot, double coralPivPosition) {
    this.coralPivot = coralPivot;
    this.coralPivPosition = coralPivPosition;

    addRequirements(coralPivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    coralPivot.setCoralPivotPosition(coralPivPosition);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    coralPivot.setCoralPivotSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (coralPivot.getCoralPivotPositionRot() >= coralPivPosition - tolerance && coralPivot.getCoralPivotPositionRot() <= coralPivPosition + tolerance) {
      return true;
    } 
    else
      return false;
  }
} 
