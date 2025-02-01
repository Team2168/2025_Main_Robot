// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralPivot extends SubsystemBase {
  private static SparkMax pivotMotor = new SparkMax(0, MotorType.kBrushless);
  private static RelativeEncoder pivotEncoder = pivotMotor.getAlternateEncoder();
  private static SparkLimitSwitch forwardLimitSwitch = pivotMotor.getForwardLimitSwitch();
  private static SparkLimitSwitch reverseLimitSwitch = pivotMotor.getReverseLimitSwitch();
  private static SparkClosedLoopController pidController = pivotMotor.getClosedLoopController();

  private final double minuteInHundredMs = 600.0;
  private final double TICKS_PER_REV = 2048;
  private final double GEAR_RATIO = 43.66162388;
  private final int SMART_CURRENT_LIMIT = 20;
  private boolean isInverted = false;
  private IdleMode brake = IdleMode.kBrake;


  /** Creates a new CoralPivot. */
  public CoralPivot() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
