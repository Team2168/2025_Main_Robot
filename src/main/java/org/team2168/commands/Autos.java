// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands;

import org.team2168.Constants;
import org.team2168.commands.CoralManipulator.DriveCoralFlywheel;
import org.team2168.commands.CoralManipulator.DriveFlywheelUntilCoral;
import org.team2168.commands.CoralManipulator.DriveFlywheelUntilNoCoral;
import org.team2168.commands.CoralManipulator.SetCoralPivotAngle;
import org.team2168.commands.Drive.DriveToPose;
import org.team2168.commands.lift.DriveLiftHeights;
import org.team2168.subsystems.CoralFlywheel;
import org.team2168.subsystems.CoralPivot;
import org.team2168.subsystems.ExampleSubsystem;
import org.team2168.subsystems.Lift;
import org.team2168.subsystems.CoralPivot.CORAL_PIVOT_POSITION;
import org.team2168.subsystems.Lift.LiftHeights;
import org.team2168.subsystems.SwerveDrivetrain.Swerve;
import org.team2168.utils.PosesUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public final class Autos {
        /** Example static factory for an autonomous command. */
        private Swerve swerve;
        private CoralFlywheel flywheel;
        private CoralPivot pivot;
        private Lift lift;
        // private Command middleCommand = new DriveToPose(() -> swerve.getState().Pose,
        // () -> PosesUtil.transformPoseDirection(false,
        // PosesUtil.findNearestScoringPose(swerve.getState().Pose,
        // PosesUtil.getScorePositionsFromAlliance(
        // DriverStation.getAlliance().get()))),
        // swerve, () -> swerve.getState().Speeds);

        public Autos(Swerve swerve, CoralFlywheel flywheel, CoralPivot pivot, Lift lift) {
                this.swerve = swerve;
                this.flywheel = flywheel;
                this.pivot = pivot;
                this.lift = lift;
        }

        public Command middleL4() {
                return Commands.sequence(swerve.resetPosePathplanner("MiddleLeave"), Commands.parallel(
                                swerve.drivePath("MiddleLeave").beforeStarting(new WaitCommand(2)),
                                new DriveLiftHeights(lift, LiftHeights.L4.getValue()),
                                new SetCoralPivotAngle(pivot, CORAL_PIVOT_POSITION.L4.getPivotPositon()),
                                new DriveFlywheelUntilNoCoral(flywheel, -0.2).beforeStarting(new WaitCommand(4.0))));
        }

        public Command leftTwoPieceL1() {
                return Commands.sequence(
                                swerve.resetPosePathplanner("TwoCoralLeftAuto1st"),
                                Commands.parallel(swerve.drivePath("TwoCoralLeftAuto1st"),
                                                new DriveCoralAngleAndElevator(pivot, lift,
                                                                CORAL_PIVOT_POSITION.BARGE.pivotPosition,
                                                                LiftHeights.BARGE.liftHeight))
                                                .andThen(new DriveFlywheelUntilNoCoral(flywheel, 0.5)),
                                Commands.parallel(swerve.drivePath("TwoCoralLeftAuto2nd"),
                                                new DriveCoralAngleAndElevator(pivot, lift,
                                                                CORAL_PIVOT_POSITION.INTAKE.pivotPosition,
                                                                LiftHeights.INTAKE.liftHeight)
                                                                .beforeStarting(new WaitCommand(1.0)))
                                                .andThen(new DriveFlywheelUntilCoral(flywheel, 0.5)),
                                Commands
                                                .parallel(swerve.drivePath("TwoCoralLeftAuto3rd"),
                                                                new DriveCoralAngleAndElevator(pivot, lift,
                                                                                CORAL_PIVOT_POSITION.BARGE.pivotPosition,
                                                                                LiftHeights.BARGE.liftHeight)))
                                .andThen(new DriveFlywheelUntilNoCoral(flywheel, 0.5));
        }

        public Command rightTwoPieceL1() {
                return Commands.sequence(
                                swerve.resetPosePathplanner("TwoCoralRightAuto1st"),
                                Commands.parallel(swerve.drivePath("TwoCoralRightAuto1st"),
                                                new DriveCoralAngleAndElevator(pivot, lift,
                                                                CORAL_PIVOT_POSITION.BARGE.pivotPosition,
                                                                LiftHeights.BARGE.liftHeight))
                                                .andThen(new DriveFlywheelUntilNoCoral(flywheel, 0.5)),
                                Commands.parallel(swerve.drivePath("TwoCoralRightAuto2nd"),
                                                new DriveCoralAngleAndElevator(pivot, lift,
                                                                CORAL_PIVOT_POSITION.INTAKE.pivotPosition,
                                                                LiftHeights.INTAKE.liftHeight)
                                                                .beforeStarting(new WaitCommand(1.0)))
                                                .andThen(new DriveFlywheelUntilCoral(flywheel, 0.5)),
                                Commands
                                                .parallel(swerve.drivePath("TwoCoralRightAuto3rd"),
                                                                new DriveCoralAngleAndElevator(pivot, lift,
                                                                                CORAL_PIVOT_POSITION.BARGE.pivotPosition,
                                                                                LiftHeights.INTAKE.liftHeight)))
                                .andThen(new DriveFlywheelUntilNoCoral(flywheel, 0.5));
        }

        public Command leftTwoPieceL4() {
                return Commands.sequence(
                                swerve.resetPosePathplanner("TwoCoralLeftAuto1st"),
                                swerve.drivePath("TwoCoralLeftAuto1st"),
                                swerve.drivePath("TwoCoralLeftAuto2nd"),
                                swerve.drivePath("TwoCoralLeftAuto3rd"));
        }

        public Command rightTwoPieceL4() {
                return Commands.sequence(
                                swerve.resetPosePathplanner("TwoCoralRightAuto1st"),
                                swerve.drivePath("TwoCoralRightAuto1st"),
                                swerve.drivePath("TwoCoralRightAuto2nd"),
                                swerve.drivePath("TwoCoralRightAuto3rd"));
        }

        public Command middleLeave() {
                return Commands.sequence(
                                swerve.resetPosePathplanner("MiddleLeave"),
                                swerve.drivePath("MiddleLeave"));
        }

        public Command leftLeave() {
                return Commands.sequence(
                                swerve.resetPosePathplanner("LeftLeave"),
                                swerve.drivePath("LeftLeave"));
        }

        public Command rightLeave() {
                return Commands.sequence(
                                swerve.resetPosePathplanner("RightLeave"),
                                swerve.drivePath("RightLeave"));
        }

        public Command middleScore() {
                return Commands.sequence(
                                swerve.resetPosePathplanner("MiddleLeave"),
                                Commands.parallel(swerve.drivePath("MiddleLeave"),
                                                new DriveLiftHeights(lift, LiftHeights.BARGE.getValue()),
                                                new SetCoralPivotAngle(pivot,
                                                                CORAL_PIVOT_POSITION.BARGE.getPivotPositon())))
                                .andThen(new DriveFlywheelUntilNoCoral(flywheel, 0.4));
        }

        public Command leftScoreSingle() {
                return Commands.sequence(
                                swerve.resetPosePathplanner("TwoCoralLeftAuto1st"),
                                Commands.parallel(swerve.drivePath("TwoCoralLeftAuto1st"),
                                                new DriveLiftHeights(lift, LiftHeights.BARGE.getValue())
                                                                .withTimeout(1.5),
                                                new SetCoralPivotAngle(pivot,
                                                                CORAL_PIVOT_POSITION.BARGE.getPivotPositon())
                                                                .withTimeout(2.0)
                                                                .andThen(Commands.parallel(
                                                                                new DriveCoralFlywheel(flywheel, 0.5),
                                                                                new SetCoralPivotAngle(pivot,
                                                                                                CORAL_PIVOT_POSITION.BARGE
                                                                                                                .getPivotPositon())))));
        }

        public Command rightScoreSingle() {
                return Commands.sequence(
                                swerve.resetPosePathplanner("TwoCoralRightAuto1st"),
                                Commands.parallel(swerve.drivePath("TwoCoralRightAuto1st"),
                                                new DriveLiftHeights(lift, LiftHeights.BARGE.getValue())
                                                                .withTimeout(1.5),
                                                new SetCoralPivotAngle(pivot,
                                                                CORAL_PIVOT_POSITION.BARGE.getPivotPositon())
                                                                .withTimeout(2.0)
                                                                .andThen(Commands.parallel(
                                                                                new DriveCoralFlywheel(flywheel, 0.5),
                                                                                new SetCoralPivotAngle(pivot,
                                                                                                CORAL_PIVOT_POSITION.BARGE
                                                                                                                .getPivotPositon())))));
        }

        public Command leftScoreSingleL4() {
                return Commands.sequence(
                                swerve.resetPosePathplanner("TwoCoralLeftAuto1st"),
                                Commands.parallel(swerve.drivePath("TwoCoralLeftAuto1st"),
                                                new DriveLiftHeights(lift, LiftHeights.L4.getValue()).withTimeout(1.5),
                                                new SetCoralPivotAngle(pivot, CORAL_PIVOT_POSITION.L4.getPivotPositon())
                                                                .withTimeout(2.0)
                                                                .andThen(Commands.parallel(
                                                                                new DriveCoralFlywheel(flywheel, -0.5),
                                                                                new SetCoralPivotAngle(pivot,
                                                                                                CORAL_PIVOT_POSITION.L4
                                                                                                                .getPivotPositon())))));
        }

        public Command rightScoreSingleL4() {
                return Commands.sequence(
                                swerve.resetPosePathplanner("TwoCoralRightAuto1st"),
                                Commands.parallel(swerve.drivePath("TwoCoralRightAuto1st"),
                                                new DriveLiftHeights(lift, LiftHeights.L4.getValue()).withTimeout(1.5),
                                                new SetCoralPivotAngle(pivot, CORAL_PIVOT_POSITION.L4.getPivotPositon())
                                                                .withTimeout(2.0)
                                                                .andThen(new WaitCommand(0.3))
                                                                .andThen(Commands.parallel(
                                                                                new DriveCoralFlywheel(flywheel, -0.5),
                                                                                new SetCoralPivotAngle(pivot,
                                                                                                CORAL_PIVOT_POSITION.L4
                                                                                                                .getPivotPositon())))));
        }

        public Command rightScoreSingleL4TEST() {
                return Commands.sequence(
                                swerve.resetPosePathplanner("TwoCoralRightAuto1st"),
                                Commands.parallel(swerve.drivePath("TwoCoralRightAuto1st"),
                                                new SetCoralPivotAngle(pivot, CORAL_PIVOT_POSITION.L4.getPivotPositon())
                                                                .withTimeout(2.0)
                                                                .alongWith(Commands
                                                                                .parallel(new WaitCommand(0.2)
                                                                                                .andThen(new DriveLiftHeights(
                                                                                                                lift,
                                                                                                                LiftHeights.L4.getValue())
                                                                                                                .withTimeout(1.5)))
                                                                                .andThen(Commands.parallel(
                                                                                                new DriveCoralFlywheel(
                                                                                                                flywheel,
                                                                                                                0.5),
                                                                                                new SetCoralPivotAngle(
                                                                                                                pivot,
                                                                                                                CORAL_PIVOT_POSITION.L4
                                                                                                                                .getPivotPositon()))))));
        }
}
