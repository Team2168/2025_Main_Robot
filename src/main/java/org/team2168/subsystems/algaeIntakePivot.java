// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.subsystems;

import org.team2168.Constants.CANDevices;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.AlternateEncoderConfig;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.annotations.Log;

public class algaeIntakePivot extends SubsystemBase {
  /** Creates a new algaeIntake. */
  //private static algaeIntakePivot instance = null;
  private final int TICKS_PER_REV = 8192;
  private final static double GEAR_RATIO = (25/1); // placeholder

  private double kP = 0.1;
  private double kI = 0.0;
  private double kD = 0.0699;

  private double maxOutput = 1;
  private double minOutput = -1;
  private boolean isInverted = true;
  private IdleMode brake = IdleMode.kBrake;
  final double MIN_ANGLE = 0.0; //in rot, change to degrees later
  final double MAX_ANGLE = -14.0;
  private double setPoint = degreesToRot(0.0);
  private final int SMART_CURRENT_LIMIT = 30; 

  private static SparkMax intakePivotOne = new SparkMax(CANDevices.INTAKE_PIVOT, SparkLowLevel.MotorType.kBrushless);
  private static SparkMaxConfig config = new SparkMaxConfig();
  private static SoftLimitConfig softLimitConfig = new SoftLimitConfig();
  private static RelativeEncoder pivotEncoder = intakePivotOne.getAlternateEncoder();
  private static AlternateEncoderConfig encoderConfig = new AlternateEncoderConfig();
  private static SparkClosedLoopController maxPid = intakePivotOne.getClosedLoopController();
  private static MAXMotionConfig motionConfig = new MAXMotionConfig();

  //neo motor
  public algaeIntakePivot() {
    config
    .inverted(isInverted)
    .idleMode(brake)
    .smartCurrentLimit(SMART_CURRENT_LIMIT);
    config.closedLoop
    .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
    .pid(kP, kI, kD)
    .outputRange(minOutput, maxOutput);
  motionConfig
    .maxVelocity(1.0);
  softLimitConfig
    .forwardSoftLimit(MIN_ANGLE) 
    .forwardSoftLimitEnabled(true)
    .reverseSoftLimit(MAX_ANGLE)
    .reverseSoftLimitEnabled(true);
  config.alternateEncoder
    .countsPerRevolution(TICKS_PER_REV)
    .positionConversionFactor(GEAR_RATIO)
    .setSparkMaxDataPortConfig()
    .inverted(isInverted)
    .apply(encoderConfig);

//configs are done minus placeholders and potential troubleshooting that arises

  config.signals.externalOrAltEncoderPosition(5);

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
    intakePivotOne.set(percentOutput);
  }

  // change to degrees later
  public void setIntakePivotPosition(double rot) {
    pivotEncoder.setPosition(rot);
  }

  // change to degrees later
  public void setIntakePivotAngle(double rot) {
    maxPid.setReference(rot, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  @Log(name = "Intake pivot angle (in degrees)")
  public double getIntakePivotAngle() {
    return pivotEncoder.getPosition();
  }

  //make a method related to angle of pivot ?

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("algae pivot (rot)", getIntakePivotAngle());
    SmartDashboard.putNumber("algae pivot (deg)", rotToDegrees(getIntakePivotAngle()));
  }
}
