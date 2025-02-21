// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.subsystems;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.AlternateEncoderConfig;
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
  //private static RelativeEncoder pivotPrimaryEncoder = pivotMotor.getEncoder();
  private static RelativeEncoder pivotEncoder = pivotMotor.getAlternateEncoder();
  private static SparkClosedLoopController pidController = pivotMotor.getClosedLoopController();

  private static DigitalInput limitSwitch = new DigitalInput(CANDevices.CORAL_PIVOT_LS);


  private final double TICKS_PER_REV = 4096;
  private static final double GEAR_RATIO = 55.55556;
  private final int SMART_CURRENT_LIMIT = 50;
  private final double MAX_ANGLE = -21.0; //TODO: is in rotations
  private final double MIN_ANGLE = 0.0;
  private boolean isInverted = false;
  private IdleMode brake = IdleMode.kBrake;
  private double kMaxOutput = 1.0;
  private double kMinOutput = -1.0;
  private int kV = 917;
  private double setPoint = 0.0;
  private double kP = 0.0145; // TODO: maybe tune values a little more
  private double kI = 0.0;
  private double kD = 0.0099;


  public enum CORAL_PIVOT_POSITION {
    BARGE(0.0),
    L2(-5.0),
    L3(0.0),
    L4(0.0);

    public double pivotPosition;

    CORAL_PIVOT_POSITION(double pivotPosition) {
      this.pivotPosition = pivotPosition;
    }

    public double getPivotPositon() {
      return pivotPosition;
    }
  }

  /** Creates a new CoralPivot. */
  public CoralPivot() {
    final SparkMaxConfig motorConfigs = new SparkMaxConfig();
    // final EncoderConfig encoderConfig = new EncoderConfig();
    final AlternateEncoderConfig altEncoderConfig = new AlternateEncoderConfig();
    pidController.setReference(setPoint, ControlType.kPosition);

    motorConfigs
      .idleMode(brake)
      .inverted(isInverted)
      .smartCurrentLimit(SMART_CURRENT_LIMIT);
    
    motorConfigs.closedLoop
      .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
      .pid(kP, kI, kD)
      //.velocityFF(1/kV)
      .outputRange(kMinOutput, kMaxOutput);

    motorConfigs.softLimit
      .forwardSoftLimit(-18.0) // in rotations
      .reverseSoftLimit(0.0) // in rotations
      .forwardSoftLimitEnabled(true)
      .reverseSoftLimitEnabled(true);

    // motorConfigs.encoder
    //   .apply(encoderConfig);

    motorConfigs.alternateEncoder
      .apply(altEncoderConfig)
      .countsPerRevolution(4096)
      .setSparkMaxDataPortConfig();

    motorConfigs.signals.externalOrAltEncoderPosition(5);

    pivotMotor.configure(motorConfigs, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }


  /**
   * sets the coral pivot's position in rotations
   * @param rot amount of rotations to move
   */
  public void setCoralPivotPosition(double rot) {
    // rot = MathUtil.clamp(rot, MIN_ANGLE, MAX_ANGLE);
    // setPoint = rot;
    pidController.setReference(rot, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    if (rot > 0) {
      if (limitSwitch.get()) {
        pivotMotor.set(0.0);
      }
      else {
        pidController.setReference(rot, ControlType.kPosition, ClosedLoopSlot.kSlot0);
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
   * @param degrees amount of degrees/angles to move pivot up
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
  public double getCoralPivotPositionDegrees() {
    return rotToDegrees(pivotEncoder.getPosition());
  }

  /**
   * gets the coral pivot's position
   * @return the position in rotations
   */
  @Log(name = "coral pivot angle (rotations)", rowIndex = 0, columnIndex = 0)
  public double getCoralPivotPositionRot() {
    return pivotEncoder.getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
