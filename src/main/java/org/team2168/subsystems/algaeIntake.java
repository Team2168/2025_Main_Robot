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

  private double kP = 15.0; //placeholders
  private double kI = 0.0;
  private double kD = 0.3;
  private double kG = -2.6;

  //private double kV = 0.12;
  //private double kA = 0.1;

  private final boolean ENABLE_CURRENT_LIMIT = true;
  private final double CONTINUOUS_CURRENT_LIMIT = 15.0;
  private final double TRIGGER_THRESHOLD_LIMIT = 20;
  private final double TRIGGER_THRESHOLD_TIME = 0.2;
  private double neutralDeadband = 0.01;
  private double maxForwardOutput = 1;
  private double maxBackwardOutput = -1;
  final double MIN_ANGLE = -120;
  final double MAX_ANGLE = 0;


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

  public void setSpeed(double percentOutput) {
    //algaeIntake.set(percentOutput);
  }

  public static algaeIntake getInstance() {
    if(instance == null)
    instance = new algaeIntake();
    return instance;
  }
}
