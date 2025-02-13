// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import org.team2168.Constants.CANDevices;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.annotations.Log;

public class CoralPivot extends SubsystemBase {
  private static SparkMax pivotMotor = new SparkMax(CANDevices.CORAL_PIVOT, MotorType.kBrushless);
  private static RelativeEncoder pivotEncoder = pivotMotor.getAlternateEncoder();
  private static SparkClosedLoopController pidController = pivotMotor.getClosedLoopController();

  private static DigitalInput limitSwitch = new DigitalInput(CANDevices.CORAL_PIVOT_LS);


  private final double TICKS_PER_REV = 2048;
  private static final double GEAR_RATIO = 43.66162388;
  private final int SMART_CURRENT_LIMIT = 20;
  private final double MAX_ANGLE = degreesToRot(0.0); //TODO: find angles
  private final double MIN_ANGLE = degreesToRot(0.0); 
  private boolean isInverted = false;
  private IdleMode brake = IdleMode.kBrake;
  private double kMaxOutput = 1.0;
  private double kMinOutput = -1.0;
  private int kV = 917;
  private double setPoint = degreesToRot(0.0); //TODO: find what setpoint is
  private double kP = 0.0; // TODO: find out PID values
  private double kI = 0.0;
  private double kD = 0.0;


  public enum CORAL_PIVOT_ANGLE {
    BARGE(0.0),
    L2(0.0),
    L3(0.0),
    L4(0.0);

    public double pivotAngle;

    CORAL_PIVOT_ANGLE(double pivotAngle) {
      this.pivotAngle = pivotAngle;
    }
  }

  /** Creates a new CoralPivot. */
  public CoralPivot() {
    final SparkMaxConfig motorConfigs = new SparkMaxConfig();
    final EncoderConfig encoderConfig = new EncoderConfig();
    pidController.setReference(setPoint, ControlType.kPosition);

    motorConfigs
      .idleMode(brake)
      .inverted(isInverted)
      .smartCurrentLimit(SMART_CURRENT_LIMIT);
    
    motorConfigs.closedLoop
      .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
      .pid(kP, kI, kD)
      .velocityFF(1/kV)
      .outputRange(kMinOutput, kMaxOutput);

    motorConfigs.softLimit
      .forwardSoftLimit(degreesToRot(0.0)) //TODO: find value
      .reverseSoftLimit(degreesToRot(0.0)) //TODO: find value
      .forwardSoftLimitEnabled(true)
      .reverseSoftLimitEnabled(true);

    motorConfigs.encoder
      .apply(encoderConfig);

    motorConfigs.signals.externalOrAltEncoderPosition(5);

    pivotMotor.configure(motorConfigs, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }


  /**
   * sets the coral pivot's position in degrees
   * @param degrees amount of degrees to move
   */
  public void setCoralPivotPosition(double degrees) {
    degrees = MathUtil.clamp(degrees, MIN_ANGLE, MAX_ANGLE);
    setPoint = degreesToRot(degrees);
    if (degrees > 0) { // TODO: find limit
      if (limitSwitch.get()) {
        pivotMotor.set(0.0);
      }
      else {
        pidController.setReference(setPoint, ControlType.kPosition);
      }
    }
  }

  public void setCoralPivotStowAngle() {
    pidController.setReference(setPoint, ControlType.kPosition);
  }

  /**
   * sets the pivit's motor speed
   * @param speed the speed in percentage. value is between -1.0 and 1.0
   */
  public void setCoralPivotSpeed(double speed) {
    pivotMotor.set(speed);
  }

  /**
   * converts rotation to degrees
   * @param ticks amount of rotations
   * @return amount degrees from amount of rotations
   */
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

  /**
   * gets the coral pivot's position
   * @return the position in degrees
   */
  @Log(name = "coral pivot angle (degrees)", rowIndex = 0, columnIndex = 0)
  public double getCoralPivotAngle() {
    return rotToDegrees(pivotEncoder.getPosition());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
