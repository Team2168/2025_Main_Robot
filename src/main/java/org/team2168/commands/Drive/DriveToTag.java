// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.Drive;

import java.util.function.Supplier;

import org.team2168.Constants;
import org.team2168.subsystems.SwerveDrivetrain.Swerve;
import org.team2168.utils.LimelightHelpers;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Rotation2d;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveToTag extends Command {
  Supplier<Pose2d> robotPose;
  Supplier<Pose2d> targetPose;
  Supplier<ChassisSpeeds> currentSpeeds;
  ProfiledPIDController xController;// Tune
  ProfiledPIDController yController;// Tune
  ProfiledPIDController thetaController; // Tune
  SwerveRequest.ApplyRobotSpeeds robotSpeeds = new SwerveRequest.ApplyRobotSpeeds();
  Swerve swerve;
  boolean left;

  public DriveToTag(Supplier<Pose2d> robotPose, Swerve swerve,
      Supplier<ChassisSpeeds> currentSpeeds, boolean left) {
    this.robotPose = robotPose;
    this.currentSpeeds = currentSpeeds;
    this.swerve = swerve;
    xController = new ProfiledPIDController(1, 0, 0,
        new TrapezoidProfile.Constraints(Constants.DrivePIDConstants.xMaxVelocity,
            Constants.DrivePIDConstants.yMaxAcceleration));
    yController = new ProfiledPIDController(1, 0, 0,
        new TrapezoidProfile.Constraints(Constants.DrivePIDConstants.yMaxVelocity,
            Constants.DrivePIDConstants.yMaxAcceleration));
    thetaController = new ProfiledPIDController(4, 0.0, 0.02, new TrapezoidProfile.Constraints(
        Constants.DrivePIDConstants.thetaMaxVelocity, Constants.DrivePIDConstants.thetaMaxAcceleration));
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // Set tolerances for the controllers
    xController.setTolerance(0.01);
    yController.setTolerance(0.01);
    thetaController.setTolerance(Units.degreesToRadians(1));
    this.left = left;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Pose2d currentPose = robotPose.get();
    thetaController.reset(currentPose.getRotation().getRadians());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d currentPose = robotPose.get();
    Pose2d target = LimelightHelpers.getTargetPose3d_CameraSpace("").toPose2d();
    Pose2d scorePose = left
        ? target.transformBy(new Transform2d(new Translation2d(0.0, 1.75), target.getRotation().unaryMinus()))
        : target.transformBy(new Transform2d(new Translation2d(0.0, -1.75), target.getRotation().unaryMinus()));

    var xSpeed = xController.calculate(scorePose.getX());
    var ySpeed = yController.calculate(scorePose.getY());
    var thetaSpeed = thetaController.calculate(scorePose.getRotation().getRadians());

    var targetSpeeds = new ChassisSpeeds(xSpeed, ySpeed, thetaSpeed);
    swerve.setControl(robotSpeeds.withSpeeds(targetSpeeds));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.setControl(robotSpeeds.withSpeeds(new ChassisSpeeds(0, 0, 0)));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return xController.atGoal() && yController.atGoal() && thetaController.atGoal();

  }
}
