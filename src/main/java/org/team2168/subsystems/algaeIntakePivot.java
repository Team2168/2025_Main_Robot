// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.subsystems;

import org.team2168.Constants;
import org.team2168.Constants.CANDevices;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkBaseConfigAccessor;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.annotations.Log;

public class algaeIntakePivot extends SubsystemBase {
  /** Creates a new algaeIntake. */
  //private static algaeIntakePivot instance = null;
  private final double TICKS_PER_REV = 2048;
  private final static double GEAR_RATIO = (25/1); // placeholder

  private double kP = 15.0; //placeholders
  private double kI = 0.0;
  private double kD = 0.3;
  private double kG = -2.6;
  private int kV = 917; //placeholder

  //private double kV = 0.12;
  //private double kA = 0.1;

  private double neutralDeadband = 0.01; //some of these are placeholders
  private double maxOutput = 1;
  private double minOutput = -1;
  private boolean isInverted = false;
  private IdleMode brake = IdleMode.kBrake;
  final double MIN_ANGLE = -120; //discover min and max angles
  final double MAX_ANGLE = 0;
  private double setPoint = degreesToRot(0.0); //find what setpoint is
  private double MAXMotionAcceleration = degreesToRot(500.0);
  private double MAXMotionCruiseVelocity = degreesToRot(250.0);
  private final int SMART_CURRENT_LIMIT = 20; 

  private static SparkMax intakePivotOne = new SparkMax(CANDevices.INTAKE_PIVOT, SparkLowLevel.MotorType.kBrushless);
  private static SparkMaxConfig config = new SparkMaxConfig();
  private static SoftLimitConfig softLimitConfig = new SoftLimitConfig();
  private static MAXMotionConfig maxMotionConfig = new MAXMotionConfig();
  private static final EncoderConfig encoderConfig = new EncoderConfig();
  private static SparkClosedLoopController maxPid = intakePivotOne.getClosedLoopController();

   private static RelativeEncoder pivotEncoder = intakePivotOne.getAlternateEncoder();
   private static DigitalInput limitSwitch = new DigitalInput(CANDevices.INTAKE_PIVOT_LIMIT_SWITCH);

  //neo motor
  public algaeIntakePivot() {
    config
    .inverted(isInverted)
    .idleMode(brake)
    .smartCurrentLimit(SMART_CURRENT_LIMIT);
  config.closedLoop
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .pid(kP, kI, kD)
    .velocityFF(1/kV)
    .outputRange(minOutput, maxOutput);
  maxMotionConfig
  .maxAcceleration(MAXMotionAcceleration)
  .maxVelocity(MAXMotionCruiseVelocity);
  softLimitConfig
    .forwardSoftLimit(degreesToRot(5.0)) //5 degrees for error tolerance
    .forwardSoftLimitEnabled(true)
    .reverseSoftLimit(degreesToRot(-125.0)) //5 degrees for error tolerance
    .reverseSoftLimitEnabled(true);
  config.encoder
  .apply(encoderConfig);

//configs are done minus placeholders and potential troubleshooting that arises
    
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

  public void setIntakePivotPosition(double degrees) {
    var demand = MathUtil.clamp(degrees, MIN_ANGLE, MAX_ANGLE);
    pivotEncoder.setPosition(degreesToRot(demand)); //change "demand" to degrees?
    if (degrees > 0) { // find limit
      if (limitSwitch.get()) {
        intakePivotOne.set(0.0);
      }
      else {
        maxPid.setReference(setPoint, ControlType.kPosition);
      }
    }
  }

  public void setIntakePivotAngle() {
    maxPid.setReference(setPoint, ControlType.kPosition);
  }

  @Log(name = "Intake pivot angle (in degrees)")
  public double getIntakePivotAngle() {
    return rotToDegrees(pivotEncoder.getPosition());
  }

  //make a method related to angle of pivot ?

  /*public static algaeIntakePivot getInstance() {
    if(instance == null)
    instance = new algaeIntakePivot();
    return instance;
  }*/

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
