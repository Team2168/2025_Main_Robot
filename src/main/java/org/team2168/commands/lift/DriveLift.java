// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.lift;

import java.util.function.DoubleSupplier;

import org.team2168.subsystems.Lift;

import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveLift extends Command {
  private Lift m_Lift;
  private DoubleSupplier m_speed;
  /** Creates a new DriveLift. */
  public DriveLift(Lift L, DoubleSupplier speed) {
    m_speed = speed;  
    m_Lift = L;
    addRequirements(m_Lift);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Lift.setPercentOutput(m_speed.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Lift.setPercentOutput(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
