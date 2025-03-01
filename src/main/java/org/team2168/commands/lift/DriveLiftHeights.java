// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.lift;

import static edu.wpi.first.units.Units.Inches;

import org.team2168.subsystems.Lift;
import org.team2168.subsystems.Lift.LiftHeights;

import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveLiftHeights extends Command {

  private final Lift m_lift;
  private final LiftHeights m_liftHeights;

  public DriveLiftHeights(Lift lift, LiftHeights H) {
    m_lift = lift;

    m_liftHeights = H;
    addRequirements(lift);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_lift.setPosition(m_liftHeights.getValue());

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_lift.setPercentOutput(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
