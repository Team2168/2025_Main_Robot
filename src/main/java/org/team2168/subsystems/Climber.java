// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.subsystems;

import org.team2168.Constants.CANDevices;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {

  TalonFX motor = new TalonFX(CANDevices.CLIMBER_ID_1);
  TalonFX follower = new TalonFX(CANDevices.CLIMBER_ID_2);
  
  // The motor's inversion is such that moving clockwise is considered moving forward
  private final InvertedValue INVERSION = InvertedValue.Clockwise_Positive;
  // The deadband for the motor--the minimum percentage output it needs to be commanded to go before actually moving
  // 0.05 is an arbitrary value
  private final double NEUTRAL_DEADBAND = 0.05; 
  // The motor brake mode. Can be brake or coast
  private final NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Brake;
  // The maximum values the motor can be commanded to go, in percent
  // Values are arbitrary
  private final double MAX_FORWARD_OUTPUT = 1.0;
  private final double MIN_FORWARD_OUTPUT = 0.0; //The motor is not allowed to move backwards 😱

  private final double FORWARD_SOFT_LIMIT = 156.81;
  private final double REVERSE_SOFT_LIMIT = 0.0;

  private final double CURRENT_LIMIT = 90.0;
  private final boolean CURRENT_LIMIT_ENABLED = true;
  
  private final double kP = 1.0;
  private final double kI = 0.0;
  private final double kD = 0.0;

  private final int FEEDBACK_SENSOR = 0;
  private final int FEEDBACK_OFFSET = 457; //arbitrary number

  private final int CRUISE_VELOCITY = 50; //in rotations per second
  private final int ACCELERATION = 25; //in rotations per seconds squared
  private final double GEAR_RATIO = 342.0;

  
  
  /**
   * sets the speed of the motor smork smork
   * @param speed -1 to 1 
   */
  
  public void driveClimbMotor(double speed) {
    motor.set(speed);
  } 


  /** Creates a new Climber. */
  public Climber() {

      motor.getConfigurator().apply(new TalonFXConfiguration()); //sets the motor to its facotry default
      follower.getConfigurator().apply(new TalonFXConfiguration());
      
      follower.setControl(new Follower(motor.getDeviceID(), true));
      

      MotorOutputConfigs motorConfigs = new MotorOutputConfigs();
      CurrentLimitsConfigs currentConfigs = new CurrentLimitsConfigs();
      Slot0Configs gains = new Slot0Configs();
      FeedbackConfigs feedbackConfigs = new FeedbackConfigs();
      MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
      SoftwareLimitSwitchConfigs softLimitConfig = new SoftwareLimitSwitchConfigs();
  
      /* Motor Output Configurations */    
      motorConfigs.withInverted(INVERSION);
      motorConfigs.withNeutralMode(NEUTRAL_MODE);
      motorConfigs.withDutyCycleNeutralDeadband(NEUTRAL_DEADBAND);
      motorConfigs.withPeakForwardDutyCycle(MAX_FORWARD_OUTPUT);
      motorConfigs.withPeakReverseDutyCycle(MIN_FORWARD_OUTPUT);
      
      /* Current Limits Configurations */
      currentConfigs.withSupplyCurrentLimit(CURRENT_LIMIT);
      currentConfigs.withSupplyCurrentLimitEnable(CURRENT_LIMIT_ENABLED);
  
      /* PID Gains Configurations */
      //setting gains with the dot operator
      gains.withKP(kP)
           .withKI(kI)
           .withKD(kD);    
  
      /* Feedback Configurations */
      feedbackConfigs.withFeedbackRemoteSensorID(FEEDBACK_SENSOR); //normally, the parameter should be calling the sensor ID from Constants
                                                                  //ex) CanDevices.SensorOne  
      feedbackConfigs.withFeedbackRotorOffset((double)FEEDBACK_OFFSET); //example of typecasting, although it is not necessary in this case
                                                                       //(Java can automatically change ints to doubles)
      
      /* Motion Magic Configurations */
      motionMagicConfigs.withMotionMagicCruiseVelocity(CRUISE_VELOCITY);
      motionMagicConfigs.withMotionMagicAcceleration(ACCELERATION);

      softLimitConfig
        .withForwardSoftLimitThreshold(FORWARD_SOFT_LIMIT)
        .withForwardSoftLimitEnable(true)
        .withReverseSoftLimitThreshold(REVERSE_SOFT_LIMIT)
        .withReverseSoftLimitEnable(true);
  
      /* Apply Configurations */
      motor.getConfigurator().apply(motorConfigs);
      motor.getConfigurator().apply(currentConfigs);
      motor.getConfigurator().apply(gains);
      motor.getConfigurator().apply(feedbackConfigs);
      motor.getConfigurator().apply(motionMagicConfigs);
      motor.getConfigurator().apply(softLimitConfig);
        
  }

  public boolean isClimbing() {
    if (motor.getPosition().getValueAsDouble() >=  FORWARD_SOFT_LIMIT || motor.getPosition().getValueAsDouble() >= (FORWARD_SOFT_LIMIT - 1.0)) {
      return true;
    }
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
