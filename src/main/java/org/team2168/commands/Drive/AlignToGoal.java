// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.Drive;

import java.util.function.Supplier;

import org.team2168.subsystems.SwerveDrivetrain.Swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AlignToGoal extends SequentialCommandGroup {
  Swerve swerve;
  Supplier<Pose2d> robotPose;
  Supplier<Pose2d> goalPose;
  boolean left;
  public AlignToGoal(Swerve swerve, Supplier<Pose2d> robotPose, Supplier<Pose2d> goalPose, boolean left) {
    this.swerve = swerve;
    this.robotPose = robotPose;
    this.goalPose = goalPose;
    this.left = left;
    addCommands(new DriveToPose(robotPose, goalPose, swerve, () -> swerve.getState().Speeds), new DriveToTag(robotPose, swerve, () -> swerve.getState().Speeds, left));
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands();
  }
}
