// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.subsystems;

import org.team2168.Constants.LiftConstants;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs; //TODO configure MotionMagic configurations
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage; 
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.annotations.Log;

public class Lift extends SubsystemBase {
  final MotionMagicVoltage m_motmag = new MotionMagicVoltage(0); // TODO Should be able to input a position
  private final double TICKS_PER_REV = 2048;
  private final double GEAR_RATIO = 0; //TODO ask CAD
  private final double INCHES_PER_REV = 0; //TODO ask somebody

  TalonFX motor = new TalonFX(LiftConstants.motorPort);
  private final InvertedValue INVERSION = InvertedValue.Clockwise_Positive; // "inversion" is placeholder
  private final NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Brake;

  private final double CURRENT_LIMIT = 20.0; //ask electrical
  private final boolean CURRENT_LIMIT_ENABLED = true;
  
  private final double kP = 0.0; //TODO tune gains
  private final double kI = 0.0; //TODO
  private final double kD = 0.0; //TODO
  private final double kArbitryFeedFoward = 0.0; //gravity accountment


  private final int CRUISE_VELOCITY = 25; //in rotations per second TODO ask elecrical
  private final int ACCELERATION = 25; //in rotations per seconds squared TODO ask electrical
  
    
  
  private void configureMotors() {
    // Set motors to factory defaults
    motor.getConfigurator().apply(new TalonFXConfiguration());
    
    MotorOutputConfigs motorConfigs = new MotorOutputConfigs();
    CurrentLimitsConfigs currentConfigs = new CurrentLimitsConfigs();
    Slot0Configs gains = new Slot0Configs();
    FeedbackConfigs feedbackConfigs = new FeedbackConfigs();
    MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();

    /* Motor Output Configurations */    
    motorConfigs.withNeutralMode(NEUTRAL_MODE);
    motorConfigs.withInverted(INVERSION); 
    /* Current Limits Configurations */
    currentConfigs.withSupplyCurrentLimit(CURRENT_LIMIT);
    currentConfigs.withSupplyCurrentLimitEnable(CURRENT_LIMIT_ENABLED);

    /* PID Gains Configurations */
    gains.withKP(kP)
         .withKI(kI)
         .withKD(kD); 


    /* Feedback Configurations */
    feedbackConfigs.withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor);
    
    /* Motion Magic Configurations */
    motionMagicConfigs.withMotionMagicCruiseVelocity(CRUISE_VELOCITY);
    motionMagicConfigs.withMotionMagicAcceleration(ACCELERATION);

    /* Apply Configurations */
    motor.getConfigurator().apply(motorConfigs);
    motor.getConfigurator().apply(currentConfigs);
    motor.getConfigurator().apply(gains);
    motor.getConfigurator().apply(feedbackConfigs);
    motor.getConfigurator().apply(motionMagicConfigs);

    
  }
  /** Creates a new Lift. */
  public Lift() {
    configureMotors();
  }
  
  public double degreesToTicks(double degrees){
    return (degrees / 360 * TICKS_PER_REV);
  }

  public double ticksToDegrees(double rotations){
    return (rotations * 360);
  }

  public double inchesToRotations(double inches){
    return(inches / INCHES_PER_REV) * GEAR_RATIO;
  }

  public  double RotationsToInches(double rotations){
    return (rotations / GEAR_RATIO) * INCHES_PER_REV;
  }

  @Log(name = "Encoder Position in Rotations")
  private double getEncoderPositionInRotations(){
    return motor.getPosition().getValueAsDouble();
  }

  public void setEncoderPosZero(){
    motor.setPosition(0);
  }

  public void setMotorBrake() {
    motor.setNeutralMode(NeutralModeValue.Brake);
  }

  public void setMotorCoast() {
    motor.setNeutralMode(NeutralModeValue.Coast);
  }

  //Config()
  public void setSpeedVelocity(double speed) {
    motor.set(ControlModeValue.Velocity, inchesToTicks(speed) * TIME_UNITS_OF_VELOCITY, DemandType.ArbitraryFeedForward, kArbitraryFeedForward); //the "speed" parameter is the rate of the movement per second (in inches)
  }

  //@Config()

  
  public void setPosition(double inches){
    //this.position = position;
  m_motmag.Slot = 0;
  motor.setControl(m_motmag.withPosition(inchesToRotations(inches))); // default position
        // motor.set(ControlModeValue.MotionMagic, inchesToRotations(inches), DemandType.ArbitraryFeedForward, kArbitraryFeedForward);
  }

  //@Config()
  public void setPercentOutput(double percentOutput) {
    motor.set(ControlModeValue.PercentOutput, percentOutput, DemandType.ArbitraryFeedForward, kArbitraryFeedForward);
  }

  public void setToZero(){
    motor.set(ControlModeValue.PercentOutput, 0, DemandType.ArbitraryFeedForward, kArbitraryFeedForward);
  }

  public void extendLock(){
    carriageLock.set(DoubleSolenoid.Value.kForward);
  }

  public void retractLock(){
    carriageLock.set(DoubleSolenoid.Value.kReverse);
  }

  @Log(name = "Positiion (inches)", rowIndex = 3, columnIndex = 2)
  public double getPositionIn(){
    return ticksToInches(motor.getSelectedSensorPosition());
  }

  @Log(name = "Speed", rowIndex = 3, columnIndex = 3)
  public double getSpeed(){
    return motor.get();
  }

  @Log(name = "Velocity (inches / sec)", rowIndex = 3, columnIndex = 4)
  public double getVelocity(){
    return (ticksToInches(motor.getSelectedSensorVelocity())) / TIME_UNITS_OF_VELOCITY;
  }

  @Log(name = "At Zero", rowIndex = 3, columnIndex = 0)
  public boolean isZeroPosition(){
    return motor.isRevLimitSwitchClosed() == 1;
  }

  @Log(name = "At Top", rowIndex = 3, columnIndex = 1)
  public boolean isAtUpperPosition(){
    return motor.isFwdLimitSwitchClosed() == 1;
  }

  @Log(name = "Error", rowIndex = 3, columnIndex = 5)
  public double getControllerError(){
    return ticksToDegrees(motor.getClosedLoopError()); //this method returns the current error position
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
