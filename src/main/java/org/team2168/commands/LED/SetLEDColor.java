// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.LED;

import org.team2168.subsystems.LEDs;

import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetLEDColor extends Command {
  /** Creates a new SetRainbowLED. */
  private LEDs leds;
  private double speed;

  public SetLEDColor(LEDs leds, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.leds = leds;
    this.speed = speed;

    addRequirements(leds);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    leds.setLEDColor(speed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    leds.setLEDColor(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
