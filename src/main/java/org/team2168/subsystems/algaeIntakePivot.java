// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.subsystems;


import org.team2168.Constants;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkBaseConfigAccessor;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class algaeIntakePivot extends SubsystemBase {
  /** Creates a new algaeIntake. */
  private static algaeIntakePivot instance = null;
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
  private boolean isInverted = false;
  private IdleMode brake = IdleMode.kBrake;
  final double MIN_ANGLE = -120;
  final double MAX_ANGLE = 0;
  final MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0.0);

  private static SparkMax intakePivotOne = new SparkMax(1, SparkLowLevel.MotorType.kBrushless);
  private static SparkMaxConfig config = new SparkMaxConfig();
  private static SparkClosedLoopController maxPid = intakePivotOne.getClosedLoopController();

   private static RelativeEncoder pivotEncoder = intakePivotOne.getAlternateEncoder();
  private static SparkLimitSwitch forwardLimitSwitch = intakePivotOne.getForwardLimitSwitch();
  private static SparkLimitSwitch reverseLimitSwitch = intakePivotOne.getReverseLimitSwitch();

  //neo motor
  public algaeIntakePivot() {
    config
    .inverted(isInverted)
    .idleMode(brake);
  config.encoder
    .positionConversionFactor(1000)
    .velocityConversionFactor(1000);
  config.closedLoop
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .pid(kP, kI, kD); //finish the configs. configs are not finished yet. still need to add current limits and soft limits.

  /*config.
    .withSupplyCurrentLimitEnable(ENABLE_CURRENT_LIMIT)
    .withSupplyCurrentLimit(CONTINUOUS_CURRENT_LIMIT)
    .withSupplyCurrentThreshold(TRIGGER_THRESHOLD_LIMIT)
    .withSupplyTimeThreshold(TRIGGER_THRESHOLD_TIME);
  config
    .smartCurrentLimit*/
    
  intakePivotOne.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters); 
  }

  public static double rotToDegrees(double rot) {
    return (rot) / GEAR_RATIO * 360.0;
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

  public void setIntakePivotPosition(double degrees) {
    var demand = MathUtil.clamp(degrees, MIN_ANGLE, MAX_ANGLE);
    //intakePivotOne.setControl(motionMagicVoltage.withPosition((degreesToRot(demand)))); //figure out something that works for this with SPARKMAX
  }

  public static algaeIntakePivot getInstance() {
    if(instance == null)
    instance = new algaeIntakePivot();
    return instance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
