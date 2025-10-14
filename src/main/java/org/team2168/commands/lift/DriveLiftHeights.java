// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.lift;

import edu.wpi.first.wpilibj2.command.Command;
import org.team2168.subsystems.Lift;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveLiftHeights extends Command {
  private Lift lift;
  private double liftPosition;
  private final double TOLERANCE = 0.5;

  /** Creates a new DriveLiftTest. */
  public DriveLiftHeights(Lift lift, double liftPosition) {
    this.lift = lift;
    this.liftPosition = liftPosition;
    addRequirements(lift);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    lift.setPosition(liftPosition);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // lift.setPercentOutput(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (lift.getPostionRotations() >= liftPosition - TOLERANCE
        && lift.getPostionRotations() <= liftPosition + TOLERANCE) {
      return true;
    } else {
      return false;
    }
  }
  // return true;
}
