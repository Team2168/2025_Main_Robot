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
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class CANDevices {
    public static final int LEDs_ID = 0;

    public static final int CAGE_DETECTOR_LS_1 = 0; //placeholder
    public static final int CAGE_DETECTOR_LS_2 = 0; //placeholder

    public static final int CORAL_PIVOT_ID = 21;
    public static final int CORAL_FLYWHEEL_ID = 20;

    public static final int LINE_BREAK_SENSOR = 0;

    public static final int ELEVATOR_ID = 12;
    public static final int ELEVATOR_CANCODER_ID = 11;
 
    public static final int CLIMBER_ID = 5; //placeholder
  }

  public static class Controllers {
    public static final int DRIVER_JOYSTICK = 0;
    public static final int OPERATOR_JOYSTICK = 1;
    public static final int TEST_JOYSTICK = 2;
  }

  public static class ClimberConstants {
    public static final int RIGHT_LIMIT_SWITCH = 2; //placeholder
    public static final int LEFT_LIMIT_SWITCH = 0; //placeholder
    public static final double CLOSING_SPEED = 0;
    public static final double OPENING_SPEED = 0.4;
  }
}
