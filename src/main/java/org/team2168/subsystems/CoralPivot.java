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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.annotations.Log;

public class CoralPivot extends SubsystemBase {
  private static SparkMax pivotMotor = new SparkMax(CANDevices.CORAL_PIVOT, MotorType.kBrushless);
  private static RelativeEncoder pivotEncoder = pivotMotor.getAlternateEncoder();
  private static SparkClosedLoopController pidController = pivotMotor.getClosedLoopController();

  private final int TICKS_PER_REV = 8192;
  private static final double GEAR_RATIO = 111.11111;
  private final int SMART_CURRENT_LIMIT = 35;
  private final double MAX_ANGLE = degreesToRot(140.0); //TODO: figure out again
  private final double MIN_ANGLE = degreesToRot(0);
  private boolean isInverted = true;
  private IdleMode brake = IdleMode.kBrake;
  private double kMaxOutput = 0.6;
  private double kMinOutput = -0.6;
  private double setPoint = 0.0;
  private double kP = 0.0149;
  private double kI = 0.0; //TODO: slightly increase to see if pivot holds position better
  private double kD = 0.013;


  public enum CORAL_PIVOT_POSITION {
    BARGE(degreesToRot(39.233)),
    L2(degreesToRot(128.477)),
    L3(degreesToRot(145.376)),
    L4(degreesToRot(150.958)),
    INTAKE(18),
    ZERO(degreesToRot(0.0));

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
    final AlternateEncoderConfig altEncoderConfig = new AlternateEncoderConfig();
    pidController.setReference(setPoint, ControlType.kPosition);
    
    /* motor configs */ 
    motorConfigs
      .idleMode(brake)
      .inverted(isInverted)
      .smartCurrentLimit(SMART_CURRENT_LIMIT);
    
    /* closed loop configs */ 
    motorConfigs.closedLoop
      .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
      .pid(kP, kI, kD)
      .outputRange(kMinOutput, kMaxOutput);

    /* soft limit configs (in rotations) */ 
    motorConfigs.softLimit
      .forwardSoftLimit(MAX_ANGLE)
      .reverseSoftLimit(MIN_ANGLE)
      .forwardSoftLimitEnabled(true)
      .reverseSoftLimitEnabled(true);

    /* alt encoder configs (using a through bore encoder) */
    motorConfigs.alternateEncoder
      .apply(altEncoderConfig)
      .countsPerRevolution(TICKS_PER_REV)
      .setSparkMaxDataPortConfig()
      .positionConversionFactor(GEAR_RATIO * TICKS_PER_REV); //changed to gear ratio * ticks per rev - check if encoder displays accurate values

    motorConfigs.signals.externalOrAltEncoderPosition(5);

    pivotMotor.configure(motorConfigs, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
  }


  /**
   * sets the coral pivot's position in rotations
   * @param rot amount of rotations to move
   */
  public void setCoralPivotPosition(double rot) {
    pidController.setReference(rot, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  /**
   * sets the coral pivot's stow angle, which is 0 degrees
   */
  public void setCoralPivotStowAngle() {
    pidController.setReference(0.0, ControlType.kPosition);
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
  @Log(name = "coral pivot angle (degrees)", rowIndex = 1, columnIndex = 0)
  public double getCoralPivotPositionDegrees() {
    return rotToDegrees(pivotEncoder.getPosition());
  }

  /**
   * gets the coral pivot's position
   * @return the position in rotations
   */
  @Log(name = "coral pivot position (rotations)", rowIndex = 1, columnIndex = 1)
  public double getCoralPivotPositionRot() {
    return pivotEncoder.getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("encoder rot", getCoralPivotPositionRot());
    SmartDashboard.putNumber("encoder deg", getCoralPivotPositionDegrees());
  }
}
