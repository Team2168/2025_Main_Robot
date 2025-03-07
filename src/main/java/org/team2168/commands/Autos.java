// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands;

import org.team2168.subsystems.ExampleSubsystem;
import org.team2168.subsystems.SwerveDrivetrain.Swerve;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public final class Autos {
  /** Example static factory for an autonomous command. */
  public static Command exampleAuto(ExampleSubsystem subsystem) {
    return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));
  }

  public static Command leftTwoPiece(Swerve swerve) {
    return Commands.sequence(
        swerve.resetPosePathplanner("TwoCoralLeftAuto1st"),
        swerve.drivePath("TwoCoralLeftAuto1st"),
        swerve.drivePath("TwoCoralLeftAuto2nd"),
        swerve.drivePath("TwoCoralLeftAuto3rd"));
  }

  public static Command middleScore(Swerve swerve) {
    return Commands.sequence(
        swerve.resetPosePathplanner("MiddleLeave"),
        swerve.drivePath("MiddleLeave"));
  }

  public static Command rightTwoPiece(Swerve swerve) {
    return Commands.sequence(
        swerve.resetPosePathplanner("TwoCoralRightAuto1st"),
        swerve.drivePath("TwoCoralRightAuto1st"),
        swerve.drivePath("TwoCoralRightAuto2nd"),
        swerve.drivePath("TwoCoralRightAuto3rd"));
  }

  public static Command leftLeave(Swerve swerve) {
    return Commands.sequence(
        swerve.resetPosePathplanner("LeftLeave"),
        swerve.drivePath("LeftLeave"));
  }

  public static Command rightLeave(Swerve swerve) {
    return Commands.sequence(
        swerve.resetPosePathplanner("RightLeave"),
        swerve.drivePath("RightLeave"));
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
