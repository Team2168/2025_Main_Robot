// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168;

import static edu.wpi.first.units.Units.Kilogram;

import java.util.DoubleSummaryStatistics;
import java.util.List;
import java.util.stream.Collectors;

import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class CANDevices {
    public static final int LEDs_ID = 5; // placeholder

    public static final int CAGE_DETECTOR_LS_1 = 4; // placeholder, prev is 0
    public static final int CAGE_DETECTOR_LS_2 = 5; // placeholder, prev is 0

    public static final int CORAL_PIVOT_ID = 21;
    public static final int CORAL_FLYWHEEL_ID = 20;

    public static final int LINE_BREAK_SENSOR = 0;

    public static final int ELEVATOR_ID = 12;
    public static final int ELEVATOR_CANCODER_ID = 11;

    public static final int CLIMBER_ID = 5; // placeholder
  }

  public static class Controllers {
    public static final int DRIVER_JOYSTICK = 0;
    public static final int OPERATOR_JOYSTICK = 1;
    public static final int TEST_JOYSTICK = 2;
  }

  public static class ClimberConstants {
    public static final int RIGHT_LIMIT_SWITCH = 2; // placeholder
    public static final int LEFT_LIMIT_SWITCH = 3; // placeholder //prev is 0
    public static final double CLOSING_SPEED = 0;
    public static final double OPENING_SPEED = 0.4;
  }

  public static class PoseConstants {
    public static final AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
    public static final List<AprilTag> tags = layout.getTags();
    public static final List<Pose2d> tagPoses = tags.stream().map(tag -> tag.pose.toPose2d())
        .collect(Collectors.toList());
    public static final List<Pose2d> scorePoses = tagPoses.stream()
        .map(tagPose -> tagPose.transformBy(new Transform2d(new Translation2d(0.63865, 0), Rotation2d.fromDegrees(180))))
        .collect(Collectors.toList());
  }

  public static class DrivePIDConstants {
    public static final double xMaxVelocity = 5;
    public static final double xMaxAcceleration = 3;
    public static final double yMaxVelocity = 5;
    public static final double yMaxAcceleration = 3;
    public static final double thetaMaxVelocity = Units.degreesToRadians(360);
    public static final double thetaMaxAcceleration = Units.degreesToRadians(180);
  }
}
