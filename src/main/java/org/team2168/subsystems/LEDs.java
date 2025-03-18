// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.subsystems;

import org.team2168.Constants.CANDevices;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase {
  /** Creates a new LEDs. */
  private static Spark leds = new Spark(CANDevices.LEDs_ID);

  public enum LED_COLOR {
    RAINBOW(-0.99),
    HOT_PINK(0.57),
    RED(0.61),
    GREEN(0.73),
    BLUE(0.87),
    PURPLE(0.91),
    TWINKLES_RAINBOW(-0.55),
    CONFETTI(-0.73);
  

    public double color;

    public double getLEDColor() {
      return color;
    }

    LED_COLOR(double color) {
      this.color = color;
    } 
  }

  public LEDs() {
  }

  /**
  * sets the LED's color based on a "motor" output value (blinkin leds are programmed like a SparkMAX)
  * @param speed the speed in percentage. value is between -1.0 and 1.0
  */
  public void setLEDColor(double speed) {
      leds.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
