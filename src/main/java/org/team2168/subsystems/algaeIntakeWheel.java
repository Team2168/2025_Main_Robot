// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.subsystems;

import org.team2168.Constants.CANDevices;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class algaeIntakeWheel extends SubsystemBase {
  /** Creates a new algaeIntakeWheel. */

  private final double minuteInHundredMs = 600.0;
  private final int TICKS_PER_REV = 4096;
  private final double GEAR_RATIO = (10/1);
  private final int SMART_CURRENT_LIMIT = 20; 
  private boolean isInverted = false;
  private IdleMode coast = IdleMode.kBrake;

  private static SparkMax intakeWheelOne = new SparkMax(CANDevices.INTAKE_WHEEL, MotorType.kBrushless);
  private static SparkMaxConfig config = new SparkMaxConfig();
  private static final EncoderConfig encoderConfig = new EncoderConfig();
  private static RelativeEncoder intakeWheelEncoder = intakeWheelOne.getEncoder();
  //private static DigitalInput intakeDetector = new DigitalInput(CANDevices.LINE_BREAK_SENSOR);
  
  public algaeIntakeWheel() {
    config
    .inverted(isInverted)
    .idleMode(coast)
    .smartCurrentLimit(SMART_CURRENT_LIMIT);
 
    // config.closedLoop
    // .feedbackSensor(FeedbackSensor.kPrimaryEncoder);

    config.encoder
    .apply(encoderConfig);

    config.signals.appliedOutputPeriodMs(2);
    
    intakeWheelOne.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);  
  }

   /**
   * sets the speed in percentage
   * @param speed value is between -1.0 and 1.0
   */
     public void setRollerSpeed(double speed) {
      intakeWheelOne.set(speed);
    }

    private double RPMToTicksPerOneHundredMS(double speedRPM) {
      return (speedRPM/minuteInHundredMs) * (TICKS_PER_REV/GEAR_RATIO);
    }
    
    /**
     * converts ticks per one hundred ms to rpm
     * @param ticksPerHundredMs amount of ticks per hundred ms
     * @return amount of rpm from ticks
     */
    private double TicksPerOneHundredMSToRPM(double ticksPerHundredMs) {
      return ticksPerHundredMs * (GEAR_RATIO/TICKS_PER_REV) * minuteInHundredMs;
    }

    public double getSpeedRPM () {
      return TicksPerOneHundredMSToRPM(intakeWheelEncoder.getVelocity());
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
