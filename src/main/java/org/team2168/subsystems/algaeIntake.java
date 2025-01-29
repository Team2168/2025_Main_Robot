// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.subsystems;
//package com.revrobotics.config;

import org.team2168.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class algaeIntake extends SubsystemBase {
  /** Creates a new algaeIntake. */
  private static algaeIntake instance = null;
  private final double TICKS_PER_REV = 2048;
  private final static double GEAR_RATIO = 50.0; // placeholder

  //private static = new

  //neo motor
  public algaeIntake() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

   /**
   * converts degrees to rotations
   * @param degrees amount of degrees/angles to move intakepivot up
   * @return amount of degrees from amount of rotations
   */
  public static double degreesToRot(double degrees) {
    return (degrees/360) * GEAR_RATIO;
  }

  /**
   * converts degrees to ticks
   * @param degrees amount of degrees
   * @return amount of ticks from amount degrees
   */
  public double degreesToTicks(double degrees) {
    return (degrees/360.0) * GEAR_RATIO * TICKS_PER_REV;
  }

  public static algaeIntake getInstance() {
    if(instance == null)
    instance = new algaeIntake();
    return instance;
  }
}
