// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.Climber;

import java.util.function.DoubleSupplier;

import org.team2168.subsystems.Climber;

import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveClimber extends Command {

private final Climber climber;

private final double input;

  /** Creates a new DriveClimber. */
  public DriveClimber(Climber c, double i) {
    climber = c;
    input = i;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(c); 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   climber.driveClimbMotor(input);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.driveClimbMotor(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
