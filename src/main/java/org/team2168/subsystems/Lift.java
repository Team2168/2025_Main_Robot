// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.subsystems;

import static edu.wpi.first.units.Units.Rotations;

import org.team2168.Constants;
import org.team2168.Constants.LiftConstants;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs; //TODO configure MotionMagic configurations
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage; 
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.annotations.Log;

public class Lift extends SubsystemBase {

   public enum LiftHeights {
    BARGE(0.0),
    L2(0.0),
    L3(0.0),
    L4(5.0);

    public double liftHeight;

    LiftHeights(double liftHeight) {

      this.liftHeight = liftHeight;
    }
    public double getValue() {
      return liftHeight;
    }
  }

 // DigitalInput toplimitSwitch = new DigitalInput(LiftConstants.topLimitSwitchID);
 // DigitalInput bottomlimitSwitch = new DigitalInput(LiftConstants.bottomLimitSwitchID);
  final MotionMagicVoltage m_motmag = new MotionMagicVoltage(0); // TODO Should be able to input a position
  final DutyCycleOut dutyCycleOut = new DutyCycleOut(0);
  final VelocityVoltage velocityVoltage = new VelocityVoltage(0);
  //private final double TICKS_PER_REV = 2048; - I don't think we need to use ticks, instead we are using rotations.
  private final double GEAR_RATIO = 21;
  private final double INCHES_PER_REV = 0; //TODO ask somebody

  TalonFX motor = new TalonFX(Constants.MotorConstants.ELEVATORID);
  CANcoder cancoder = new CANcoder(Constants.MotorConstants.CANCODER_ID);
  private final InvertedValue INVERSION = InvertedValue.CounterClockwise_Positive; // "inversion" is placeholder
  private final NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Brake;
  private final GravityTypeValue FEEDFORWARD_TYPE = GravityTypeValue.Elevator_Static;

  private final SensorDirectionValue encoderValue = SensorDirectionValue.Clockwise_Positive;
  private final double peakCancoderAbsolute = 1.0;

  private final double STATOR_CURRENT_LIMIT = 40.0; //ask electrical
  private final double SUPPLY_CURRENT_LIMIT = 45.0;
  private final double SUPPLY_LOWER_LIMIT = 40;
  private final double LOWER_TIME = 1;
  private final boolean CURRENT_LIMIT_ENABLED = true;
  private final double PEAK_FORWARD_DUTY_CYCLE = 1.0;
  private final double PEAK_REVERSE_DUTY_CYCLE = -1.0;
  private final double NEUTRAL_DEADBAND = 0.005;
  
  private final double kP = 2.75; //TODO tune gains
  private final double kI = 0.0; //TODO
  private final double kD = 0.25; //TODO
  private final double kGravity = 0.24; //gravity accountment


  private final int CRUISE_VELOCITY = 85; // TODO modify in future
  private final int ACCELERATION = 50;  // TODO modify in future
  private final double expoKV = 0.119;
  private final double expoKA = 0.1;

  private final double peakForwardVoltage = 16.0;
  private final double oeakReverseVoltage = -16.0;

  private final double forwardSoftLimit = 5.23;
  private final double reverseSoftLimit = 0.0;
    
  
  private void configureMotors() {
    // Set motors to factory defaults
    motor.getConfigurator().apply(new TalonFXConfiguration());
    
    MotorOutputConfigs motorConfigs = new MotorOutputConfigs();
    CurrentLimitsConfigs currentConfigs = new CurrentLimitsConfigs();
    Slot0Configs gains = new Slot0Configs();
    FeedbackConfigs feedbackConfigs = new FeedbackConfigs();
    MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();

    /* Motor Output Configurations */   
    motorConfigs
    .withNeutralMode(NEUTRAL_MODE)
    .withInverted(INVERSION);
     
   
    /* Current Limits Configurations */
    currentConfigs
    .withSupplyCurrentLimit(CURRENT_LIMIT)
    .withSupplyCurrentLimitEnable(CURRENT_LIMIT_ENABLED);

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
  
 // public double degreesToRotations(double degrees){
 //   return (degrees / 360);
 // }

 // public double rotationsToDegrees(double rotations){
 //   return (rotations * 360);
 // }

  public double inchesToRotations(double inches){
    return(inches / INCHES_PER_REV) * GEAR_RATIO;
  }

  public  double rotationsToInches(double rotations){
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
    velocityVoltage.Slot = 0;
    motor.setControl(velocityVoltage.withVelocity(speed).withFeedForward(kArbitryFeedFoward));
   // if (speed > 0) {
   //   if (toplimitSwitch.get()) {
          // We are going up and top limit is tripped so stop
   //       motor.setControl(velocityVoltage.withVelocity(0).withFeedForward(kArbitryFeedFoward));
   //   } else {
          // We are going up but top limit is not tripped so go at commanded speed
   //       motor.setControl(velocityVoltage.withVelocity(speed).withFeedForward(kArbitryFeedFoward));
   //   }
   // } else {
   //   if (bottomlimitSwitch.get()) {
          // We are going down and bottom limit is tripped so stop
   //       motor.setControl(velocityVoltage.withVelocity(0).withFeedForward(kArbitryFeedFoward));
   //   } else {
   //       // We are going down but bottom limit is not tripped so go at commanded speed
   //       motor.setControl(velocityVoltage.withVelocity(speed).withFeedForward(kArbitryFeedFoward));
   //   }
   // }
  }
//(ControlModeValue.Velocity, inchesToRotations(speed) * TIME_UNITS_OF_VELOCITY, DemandType.ArbitraryFeedForward, kArbitraryFeedForward); //the "speed" parameter is the rate of the movement per second (in inches)
//}

  //@Config()

  public void setPosition(double rotations){
    m_motmag.Slot = 0;
    motor.setControl(m_motmag.withPosition(rotations).withFeedForward(kArbitryFeedFoward));
    
  // Limit switch code
  //  if (inches > 0) {
  //    if (toplimitSwitch.get()) {
  //      motor.setControl(m_motmag.withPosition(inchesToRotations(0)).withFeedForward(kArbitryFeedFoward));
  //    }
  //    else {
  //      motor.setControl(m_motmag.withPosition(inchesToRotations(inches)).withFeedForward(kArbitryFeedFoward));
  //    }
  //  } else {
  //    if (bottomlimitSwitch.get()) {
  //    motor.setControl(m_motmag.withPosition(inchesToRotations(0)).withFeedForward(kArbitryFeedFoward));
  //    }
  //    else {
  //      motor.setControl(m_motmag.withPosition(inchesToRotations(inches)).withFeedForward(kArbitryFeedFoward));
  //    }
  //  }
        
  }

  //@Config()
  public void setPercentOutput(double percentOutput) {
    motor.setControl(dutyCycleOut.withOutput(percentOutput));
   // if (percentOutput > 0) {
   //  if (toplimitSwitch.get()) {
   //    motor.setControl(dutyCycleOut.withOutput(0));
   //   }
   //   else {
   //     motor.setControl(dutyCycleOut.withOutput(percentOutput));
   //   }
   // } else {
   //   if (bottomlimitSwitch.get()) {
   //     motor.setControl(dutyCycleOut.withOutput(0));
   //   }
   //   else {
   //     motor.setControl(dutyCycleOut.withOutput(percentOutput));
    }
  
 
  public void setToZero(){
    motor.setControl(dutyCycleOut.withOutput(0));
  }
//(ControlModeValue.PercentOutput, 0, DemandType.ArbitraryFeedForward, kArbitraryFeedForward);

  @Log(name = "Position (inches)", rowIndex = 3, columnIndex = 2)
  public double getPositionIn(){
    return rotationsToInches(motor.getPosition().getValueAsDouble());
  }

  @Log(name = "Speed (%Output)", rowIndex = 3, columnIndex = 3)
  public double getSpeed(){
    return motor.get();
  }

  @Log(name = "Velocity (inches / sec)", rowIndex = 3, columnIndex = 4)
  public double getVelocity(){
    return (rotationsToInches(motor.getVelocity().getValueAsDouble()));
  }

  // @Log(name = "At Zero", rowIndex = 3, columnIndex = 0)
  // public boolean isZeroPosition(){
  //   return motor.isRevLimitSwitchClosed() == 1;
  // }

 // @Log(name = "At Top", rowIndex = 3, columnIndex = 1)
 // public boolean isAtUpperPosition(){
 //   return motor.isFwdLimitSwitchClosed() == 1;
 // }

  @Log(name = "Error", rowIndex = 3, columnIndex = 5)
  public double getControllerError(){
    return motor.getClosedLoopError().getValueAsDouble(); //this method returns the current error position
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
