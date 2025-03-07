// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static  class LiftConstants {
    public static final int topLimitSwitchID = 0;
    public static final int bottomLimitSwitchID = 0;
    public static final int motorPort = 0;
  }
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class CANDevices {

    public static final int INTAKE_PIVOT = 31; //placeholders
    public static final int INTAKE_WHEEL = 30;

    public static final int CORAL_PIVOT = 21;
    public static final int CORAL_FLYWHEEL = 20;

    public static final int LINE_BREAK_SENSOR = 0; // TODO: need value
  }

  public static class Controllers {
    public static final int DRIVER_JOYSTICK = 0; //placeholders
    public static final int OPERATOR_JOYSTICK = 1;
    public static final int TEST_JOYSTICK = 2;
  }



  public static class ClimberConstants {
    public static final int ClimberMotorID = 5;
    public static final double openingSpeed = 0.4;
  
  public static final int rightlimitSwitchChannel = 2;
  public static final int leftlimitSwitchChannel = 0;
  public static final double closingSpeed = 0;
  }

  public static class MotorConstants {
    public static final int ELEVATORID = 12;
    public static final int CANCODER_ID = 11;
  }
}
