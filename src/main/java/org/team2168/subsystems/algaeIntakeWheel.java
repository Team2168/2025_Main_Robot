// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.subsystems;

import org.team2168.Constants;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.annotations.Log;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class algaeIntakeWheel extends SubsystemBase {
  /** Creates a new algaeIntakeWheel. */

  private static algaeIntakeWheel instance = null;

  private final double minuteInHundredMs = 600.0;
  private final double TICKS_PER_REV = 2048;
  private final double GEAR_RATIO = 0; //placeholder
  private final int SMART_CURRENT_LIMIT = 20;
  private boolean isInverted = false;
  private IdleMode coast = IdleMode.kCoast;

  private static SparkMax intakeWheelOne = new SparkMax(1, SparkLowLevel.MotorType.kBrushless);
  private static RelativeEncoder intakeWheelEncoder = intakeWheelOne.getEncoder();


  public algaeIntakeWheel() {
    /*intakeWheelOne.restoreFactoryDefaults();

    intakeWheelOne.setInverted(isInverted); //outdated
    intakeWheelOne.setIdleMode(coast);
    intakeWheelOne.setSmartCurrentLimit(SMART_CURRENT_LIMIT);*/ 

    //figure out how to make these work in 2025 sparkmax code
    
  }
  
    public static algaeIntakeWheel getInstance() {
      if(instance == null)
      instance = new algaeIntakeWheel();
      return instance;
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
