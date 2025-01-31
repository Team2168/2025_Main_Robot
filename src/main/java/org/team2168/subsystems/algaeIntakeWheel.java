// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.subsystems;

import org.team2168.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.annotations.Log;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class algaeIntakeWheel extends SubsystemBase {
  /** Creates a new algaeIntakeWheel. */

  private static algaeIntakeWheel instance = null;

  private final double minuteInHundredMs = 600.0;
  private final double TICKS_PER_REV = 2048;
  private final double GEAR_RATIO = 0; //placeholder
  private final int SMART_CURRENT_LIMIT = 20;
  private boolean isInverted = false;

  public algaeIntakeWheel() {
    
  }
  
    public static algaeIntakeWheel getInstance() {
      if(instance == null)
      instance = new algaeIntakeWheel();
      return instance;
    }

    private double RPMToTicksPerOneHundredMS(double speedRPM) {
      return (speedRPM/minuteInHundredMs) * (TICKS_PER_REV/GEAR_RATIO);
    }
    
    /**
     * converts ticks per one hundred ms to rpm
     * @param ticksPerHundredMs amount of ticks per hundred ms
     * @return amount of rpm from ticks
     */
    private double TicksPerOneHundredMSToRPM(double ticksPerHundredMs) {
      return ticksPerHundredMs * (GEAR_RATIO/TICKS_PER_REV) * minuteInHundredMs;
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
