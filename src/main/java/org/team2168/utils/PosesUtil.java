// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.utils;

import java.util.List;

import org.team2168.Constants;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public class PosesUtil {
    public static List<Pose2d> getScorePositionsFromAlliance(Alliance alliance) {
        List<Pose2d> allianceScoringPositions = alliance == Alliance.Red
                ? Constants.PoseConstants.scorePoses.subList(5, 11)
                : Constants.PoseConstants.scorePoses.subList(16, 22);

        return allianceScoringPositions;
    }

    public static Pose2d findNearestScoringPose(Pose2d robotPose, List<Pose2d> scoringPoses) {
        return robotPose.nearest(scoringPoses);
    }

    public static Pose2d transformPoseDirection(boolean leftSide, Pose2d scoringPosition) {
        Pose2d reefSideScoringPosition = leftSide
                ? scoringPosition.transformBy(new Transform2d(new Translation2d(0, 0.2), new Rotation2d()))
                : scoringPosition.transformBy(new Transform2d(new Translation2d(0, -0.2), new Rotation2d()));
        return reefSideScoringPosition;
    }
}
