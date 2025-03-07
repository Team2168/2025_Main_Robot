// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.team2168.Constants.Controllers;
import org.team2168.Constants.OperatorConstants;
import org.team2168.Constants.ClimberConstants;

import org.team2168.commands.Autos;
import org.team2168.commands.ExampleCommand;
import org.team2168.commands.CloseClimber;
import org.team2168.commands.DriveClimber;
import org.team2168.commands.CoralManipulator.BumpCoralPivotAngleDown;
import org.team2168.commands.CoralManipulator.BumpCoralPivotAngleUp;
import org.team2168.commands.CoralManipulator.DriveCoralFlywheel;
import org.team2168.commands.CoralManipulator.DriveFlywheelUntilCoral;
import org.team2168.commands.CoralManipulator.DriveFlywheelUntilNoCoral;
import org.team2168.commands.CoralManipulator.SetCoralPivotAngle;
import org.team2168.commands.Drive.DriveToPose;
import org.team2168.commands.DriveFlywheelWithJoystick;
import org.team2168.commands.lift.DriveLift;
import org.team2168.commands.lift.DriveLiftHeights;
import org.team2168.commands.LEDStatus;

import org.team2168.subsystems.CoralFlywheel;
import org.team2168.subsystems.CoralPivot;
import org.team2168.subsystems.CoralPivot.CORAL_PIVOT_POSITION;
import org.team2168.subsystems.Climber;

import org.team2168.subsystems.ExampleSubsystem;
import org.team2168.subsystems.SwerveDrivetrain.Swerve;
import org.team2168.subsystems.SwerveDrivetrain.TunerConstants;
import org.team2168.utils.PosesUtil;
import org.team2168.utils.Telemetry;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.util.Units;
import org.team2168.subsystems.Lift;
import org.team2168.subsystems.Lift.LiftHeights;
import org.team2168.subsystems.CageDetector;
import org.team2168.subsystems.LEDs;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.LTVDifferentialDriveController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import io.github.oblarg.oblog.Logger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  SendableChooser<Command> autoChooser = new SendableChooser<>();
  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
  private final Swerve swerve = TunerConstants.createDrivetrain();
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem(); // Use open-loop control for drive motors

  private final Telemetry logger = new Telemetry(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond), swerve);

  private final CoralFlywheel coralFlywheel = new CoralFlywheel();
  private final CoralPivot coralPivot = new CoralPivot();

  private final Lift lift = new Lift();

  private final Climber climber = new Climber();

  private final LEDs leds = new LEDs();
  private final CageDetector cageDetector = new CageDetector();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  public CommandXboxController driverJoystick = new CommandXboxController(Controllers.DRIVER_JOYSTICK);
  public CommandXboxController operatorJoystick = new CommandXboxController(Controllers.OPERATOR_JOYSTICK);
  public CommandXboxController testJoystick = new CommandXboxController(Controllers.TEST_JOYSTICK);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureAutonomous();
    configureBindings();
    Logger.configureLoggingAndConfig(this, false);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  // bindings for moving the climber teehee
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // new Trigger(m_exampleSubsystem::exampleCondition)
    // .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is
    // pressed,
    // cancelling on release.
    // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());

    driverJoystick.leftTrigger().whileFalse(new InstantCommand(() -> {
      MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
      MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
    }, swerve));

    driverJoystick.leftTrigger().whileTrue(new InstantCommand(() -> {
      MaxSpeed = MaxSpeed / 8;
      MaxAngularRate = MaxAngularRate / 8;
    }, swerve));

    driverJoystick.leftBumper().onTrue(
        new DriveToPose(() -> swerve.getState().Pose,
            () -> PosesUtil.transformPoseDirection(true, PosesUtil.findNearestScoringPose(swerve.getState().Pose,
                PosesUtil.getScorePositionsFromAlliance(Alliance.Red))),
            swerve, () -> swerve.getState().Speeds));

    driverJoystick.rightBumper().onTrue(
        new DriveToPose(() -> swerve.getState().Pose,
            () -> PosesUtil.transformPoseDirection(false, PosesUtil.findNearestScoringPose(swerve.getState().Pose,
                PosesUtil.getScorePositionsFromAlliance(Alliance.Red))),
            swerve, () -> swerve.getState().Speeds));

    swerve.setDefaultCommand(
        swerve.runDriveWithJoystick(driverJoystick, MaxSpeed, MaxAngularRate));
    driverJoystick.leftTrigger()
        .onTrue(swerve.runDriveWithJoystick(driverJoystick, MaxSpeed / 10, MaxAngularRate / 10));

    swerve.registerTelemetry(logger::telemeterize);

    leds.setDefaultCommand(new LEDStatus(leds, cageDetector, coralFlywheel));

    testJoystick.rightBumper().whileTrue(new DriveFlywheelUntilCoral(coralFlywheel, -0.6));
    testJoystick.leftBumper().whileTrue(new DriveFlywheelUntilNoCoral(coralFlywheel, 0.6));

    testJoystick.rightTrigger().onTrue(new BumpCoralPivotAngleUp(coralPivot)); // test
    testJoystick.leftTrigger().onTrue(new BumpCoralPivotAngleDown(coralPivot));

    testJoystick.rightStick().whileTrue(new DriveLift(lift, () -> testJoystick.getRightY()));

    /*
     * coral shoot button: if coral is in the intake, then these will shoot it out
     * coral (cannot shoot into L1)
     */
    operatorJoystick.rightTrigger().whileTrue(new DriveFlywheelUntilNoCoral(coralFlywheel, -0.6));

    /*
     * coral intake button: sets elevator and coral pivot to the position to intake
     * coral while also running the coral flywheel until line break sensor senses
     * coral
     */
    operatorJoystick.rightBumper()
        .whileTrue((Commands.parallel(new SetCoralPivotAngle(coralPivot, CORAL_PIVOT_POSITION.INTAKE.getPivotPositon()),
            new DriveLiftHeights(lift, LiftHeights.INTAKE.getValue()),
            new DriveFlywheelUntilCoral(coralFlywheel, -0.65))));

    /*
     * control coral in intake buttons: (needs testing) sets the coral intake to a
     * very slow speed depending on if the operator moves the left stick up or down
     */
    // operatorJoystick.leftStick()
    // .whileTrue(new DriveFlywheelWithJoystick(coralflyWheel, () ->
    // operatorJoystick.getLeftY()));

    operatorJoystick.povUp().whileTrue(new DriveCoralFlywheel(coralFlywheel, 0.15));
    operatorJoystick.povDown().whileTrue(new DriveCoralFlywheel(coralFlywheel, -0.15));

    /* L1-L4 buttons: sets elevator and coral pivot to desired reef branch */
    operatorJoystick.a()
        .onTrue(Commands.parallel(new SetCoralPivotAngle(coralPivot, CORAL_PIVOT_POSITION.BARGE.getPivotPositon()),
            new DriveLiftHeights(lift, LiftHeights.BARGE.getValue())));
    operatorJoystick.b()
        .onTrue(Commands.parallel(new SetCoralPivotAngle(coralPivot, CORAL_PIVOT_POSITION.L2.getPivotPositon()),
            new DriveLiftHeights(lift, LiftHeights.L2.getValue())));
    operatorJoystick.y()
        .onTrue(Commands.parallel(new SetCoralPivotAngle(coralPivot, CORAL_PIVOT_POSITION.L3.getPivotPositon()),
            new DriveLiftHeights(lift, LiftHeights.L3.getValue())));
    operatorJoystick.x()
        .onTrue(Commands.parallel(new SetCoralPivotAngle(coralPivot, CORAL_PIVOT_POSITION.L4.getPivotPositon()),
            new DriveLiftHeights(lift, LiftHeights.L4.getValue())));

    /* reset button: sets elevator and coral pivot position to 0 */
    operatorJoystick.povRight()
        .onTrue(Commands.parallel(new SetCoralPivotAngle(coralPivot, CORAL_PIVOT_POSITION.ZERO.getPivotPositon()),
            new DriveLiftHeights(lift, LiftHeights.ZERO.getValue())));

    operatorJoystick.povLeft().whileTrue(new CloseClimber(climber));
    operatorJoystick.povRight().whileTrue(new DriveClimber(climber, () -> ClimberConstants.OPENING_SPEED));

  }

  private void configureAutonomous() {
    autoChooser.addOption("MiddleLeave", Autos.middleScore(swerve));
    autoChooser.addOption("LeftLeaveScore", Autos.leftTwoPiece(swerve));
    autoChooser.addOption("RightLeaveScore", Autos.rightTwoPiece(swerve));
    autoChooser.addOption("RightLeave", Autos.rightLeave(swerve));
    autoChooser.addOption("LeftLeave", Autos.leftLeave(swerve));
    SmartDashboard.putData("Auto Mode", autoChooser);
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    Command auto = autoChooser.getSelected();
    if (auto != null) {
      return auto;
    } else {
      return Commands.none();
    }
  }

}
