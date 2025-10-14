// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.subsystems;


import org.team2168.Constants.CANDevices;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.annotations.Log;

public class Lift extends SubsystemBase {

  public enum LiftHeights {
    BARGE(0.5148),
    L2(0.1139),
    L3(1.6365),
    L4(5.27),
    INTAKE(2.654),
    ZERO(0.0);

    public double liftHeight;

    LiftHeights(double liftHeight) {
      this.liftHeight = liftHeight;
    }

    public double getValue() {
      return liftHeight;
    }
  }



  final MotionMagicVoltage m_motmag = new MotionMagicVoltage(0);
  final DutyCycleOut dutyCycleOut = new DutyCycleOut(0);
  final VelocityVoltage velocityVoltage = new VelocityVoltage(0);
  // private final double TICKS_PER_REV = 2048; - I don't think we need to use
  // ticks, instead we are using rotations.
  private final double GEAR_RATIO = 21;
  private final double INCHES_PER_REV = 0; // TODO ask somebody

  TalonFX motor = new TalonFX(CANDevices.ELEVATOR_ID);
  CANcoder cancoder = new CANcoder(CANDevices.ELEVATOR_CANCODER_ID);

  private final InvertedValue INVERSION = InvertedValue.CounterClockwise_Positive;
  private final NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Brake;
  private final GravityTypeValue FEEDFORWARD_TYPE = GravityTypeValue.Elevator_Static;

  private final SensorDirectionValue ENCODER_DIRECTION = SensorDirectionValue.Clockwise_Positive;
  private final double PEAK_CANCODER_ABSOLUTE = 1.0;

  private final double STATOR_CURRENT_LIMIT = 40.0;
  private final double SUPPLY_CURRENT_LIMIT = 45.0;
  private final double SUPPLY_LOWER_LIMIT = 40;
  private final double LOWER_TIME = 1;
  private final boolean CURRENT_LIMIT_ENABLED = true;
  private final double PEAK_FORWARD_DUTY_CYCLE = 1.0;
  private final double PEAK_REVERSE_DUTY_CYCLE = -1.0;
  private final double NEUTRAL_DEADBAND = 0.005;

  private final double KP = 3.4; //origally 3.0
  private final double KI = 0.0;
  private final double KD = 0.9; // originally 0.23
  private final double K_Gravity = 0.24; // gravity accountment

  private final int CRUISE_VELOCITY = 160; // TODO modify in future
  private final int ACCELERATION = 130; // TODO modify in future
  private final double EXPO_KV = 0.119;
  private final double EXPO_KA = 0.1;

  private final double PEAK_FORWARD_VOLTAGE = 16.0;
  private final double PEAK_REVERSE_VOLTAGE = -16.0;

  private final double FORWARD_SOFT_LIMIT = 5.3;
  private final double REVERSE_SOFT_LIMIT = 0.0;

  private void configureMotors() {
    // Set motors to factory defaults
    motor.getConfigurator().apply(new TalonFXConfiguration());
    cancoder.getConfigurator().apply(new CANcoderConfiguration());

    MagnetSensorConfigs magnetConfigs = new MagnetSensorConfigs();
    MotorOutputConfigs motorConfigs = new MotorOutputConfigs();
    CurrentLimitsConfigs currentConfigs = new CurrentLimitsConfigs();
    Slot0Configs gains = new Slot0Configs();
    FeedbackConfigs feedbackConfigs = new FeedbackConfigs();
    MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
    SoftwareLimitSwitchConfigs softLimitConfigs = new SoftwareLimitSwitchConfigs();
    VoltageConfigs voltageConfigs = new VoltageConfigs();

    magnetConfigs.withAbsoluteSensorDiscontinuityPoint(PEAK_CANCODER_ABSOLUTE).withSensorDirection(ENCODER_DIRECTION);

    /* Motor Output Configurations */
    motorConfigs
        .withNeutralMode(NEUTRAL_MODE)
        .withInverted(INVERSION).withPeakForwardDutyCycle(PEAK_FORWARD_DUTY_CYCLE)
        .withPeakReverseDutyCycle(PEAK_REVERSE_DUTY_CYCLE).withDutyCycleNeutralDeadband(NEUTRAL_DEADBAND);

    /* Current Limits Configurations */
    currentConfigs
        .withSupplyCurrentLimit(SUPPLY_CURRENT_LIMIT)
        .withSupplyCurrentLimitEnable(CURRENT_LIMIT_ENABLED).withStatorCurrentLimit(STATOR_CURRENT_LIMIT)
        .withStatorCurrentLimitEnable(CURRENT_LIMIT_ENABLED).withSupplyCurrentLowerLimit(SUPPLY_LOWER_LIMIT)
        .withSupplyCurrentLowerTime(LOWER_TIME);

    /* PID Gains Configurations */
    gains.withKP(KP)
        .withKI(KI)
        .withKD(KD).withGravityType(FEEDFORWARD_TYPE).withKG(K_Gravity);

    /* Feedback Configurations */
    feedbackConfigs.withRemoteCANcoder(cancoder);

    /* Motion Magic Configurations */
    motionMagicConfigs.withMotionMagicCruiseVelocity(CRUISE_VELOCITY);
    motionMagicConfigs.withMotionMagicAcceleration(ACCELERATION).withMotionMagicExpo_kA(EXPO_KA)
        .withMotionMagicExpo_kV(EXPO_KV);
      
    softLimitConfigs.withForwardSoftLimitEnable(true).withForwardSoftLimitThreshold(FORWARD_SOFT_LIMIT)
        .withReverseSoftLimitEnable(true).withReverseSoftLimitThreshold(REVERSE_SOFT_LIMIT);

    voltageConfigs.withPeakForwardVoltage(PEAK_FORWARD_VOLTAGE).withPeakReverseVoltage(PEAK_REVERSE_VOLTAGE);

    /* Apply Configurations */
    cancoder.getConfigurator().apply(magnetConfigs);
    motor.getConfigurator().apply(motorConfigs);
    motor.getConfigurator().apply(currentConfigs);
    motor.getConfigurator().apply(gains);
    motor.getConfigurator().apply(feedbackConfigs);
    motor.getConfigurator().apply(motionMagicConfigs);
    motor.getConfigurator().apply(softLimitConfigs);
    motor.getConfigurator().apply(voltageConfigs);
    
  }

  /** Creates a new Lift. */
  public Lift() {
    configureMotors();
  }

  // public double degreesToRotations(double degrees){
  // return (degrees / 360);
  // }

  // public double rotationsToDegrees(double rotations){
  // return (rotations * 360);
  // }

  public double inchesToRotations(double inches) {
    return (inches / INCHES_PER_REV) * GEAR_RATIO;
  }

  public double rotationsToInches(double rotations) {
    return (rotations / GEAR_RATIO) * INCHES_PER_REV;
  }

  @Log(name = "Encoder Position in Rotations")
  private double getEncoderPositionInRotations() {
    return motor.getPosition().getValueAsDouble();
  }

  public void setEncoderPosZero() {
    motor.setPosition(0);
  }

  public void setMotorBrake() {
    motor.setNeutralMode(NeutralModeValue.Brake);
  }

  public void setMotorCoast() {
    motor.setNeutralMode(NeutralModeValue.Coast);
  }

  // Config()
  public void setSpeedVelocity(double speed) {
    velocityVoltage.Slot = 0;
    motor.setControl(velocityVoltage.withVelocity(speed));
  }

  // (ControlModeValue.Velocity, inchesToRotations(speed) *
  // TIME_UNITS_OF_VELOCITY, DemandType.ArbitraryFeedForward,
  // kArbitraryFeedForward); //the "speed" parameter is the rate of the movement
  // per second (in inches)
  // }

  // @Config()
  public void setPosition(double rotations) {
    m_motmag.Slot = 0;
    motor.setControl(m_motmag.withPosition(rotations));
  }

  // @Config()
  public void setPercentOutput(double percentOutput) {
    motor.setControl(dutyCycleOut.withOutput(percentOutput));
  }

  public void setToZero() {
    motor.setControl(dutyCycleOut.withOutput(0));
  }

  public double getPostionRotations() {
    return motor.getPosition().getValueAsDouble();
  }
  // (ControlModeValue.PercentOutput, 0, DemandType.ArbitraryFeedForward,
  // kArbitraryFeedForward);

  @Log(name = "Position (inches)", rowIndex = 3, columnIndex = 2)
  public double getPositionIn() {
    return rotationsToInches(motor.getPosition().getValueAsDouble());
  }

  @Log(name = "Speed (%Output)", rowIndex = 3, columnIndex = 3)
  public double getSpeed() {
    return motor.get();
  }

  @Log(name = "Velocity (inches / sec)", rowIndex = 3, columnIndex = 4)
  public double getVelocity() {
    return (rotationsToInches(motor.getVelocity().getValueAsDouble()));
  }

  @Log(name = "Error", rowIndex = 3, columnIndex = 5)
  public double getControllerError() {
    return motor.getClosedLoopError().getValueAsDouble(); // this method returns the current error position
  }

  public double getCanCoderPosition() {
    return cancoder.getPosition().getValueAsDouble();
  }

  public double getCanCoderAbsolutePosition() {
    return cancoder.getAbsolutePosition().getValueAsDouble();
  }
  @Override
  
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("elevator position (rot)", getPostionRotations());
    SmartDashboard.putNumber("elevator position (in)", getPositionIn());
    SmartDashboard.putNumber(" elevator cancoder (rot)", getCanCoderPosition());
    SmartDashboard.putNumber("elevator absolute cancoder (rot)", getCanCoderAbsolutePosition());
  }
}
