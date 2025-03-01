// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.subsystems;

import org.team2168.Constants.CANDevices;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CageDetector extends SubsystemBase {
  /** Creates a new CageDetector. */
  DigitalInput limitSwitch1 = new DigitalInput(CANDevices.CAGE_DETECTOR_LS_1);
  DigitalInput limitSwitch2 = new DigitalInput(CANDevices.CAGE_DETECTOR_LS_2);

  public CageDetector() {}

  public boolean canClimb() {
    if (limitSwitch1.get() && limitSwitch2.get()) {
      return true;
    }
    else return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Can Climb?", canClimb());
  }
}
