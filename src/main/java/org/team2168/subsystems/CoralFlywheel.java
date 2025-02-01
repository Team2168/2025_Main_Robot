package org.team2168.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralFlywheel extends SubsystemBase {
    private static SparkMax flywheelMotor = new SparkMax(0, MotorType.kBrushless); // placeholder ID
    private static RelativeEncoder flywheelEncoder = flywheelMotor.getAlternateEncoder();

    private final double minuteInHundredMs = 600.0;
    private final double TICKS_PER_REV = 2048;
    private final double GEAR_RATIO = 50;
    private final int SMART_CURRENT_LIMIT = 20;
    private boolean isInverted = false;
    private IdleMode coast = IdleMode.kCoast;

    private CoralFlywheel() {
        final SparkMaxConfig motorConfigs = new SparkMaxConfig();

        motorConfigs
            .idleMode(coast)
            .inverted(isInverted)
            .smartCurrentLimit(SMART_CURRENT_LIMIT);

        motorConfigs.signals.primaryEncoderPositionPeriodMs(5);

        flywheelMotor.configure(motorConfigs, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

     /**
     * sets the flywheel's motor speed
     * @param speed the speed in percentage. value is between -1.0 and 1.0
     */
    public void setFlywheelSpeed(double speed) {
        flywheelMotor.set(speed);
    }

     /**
     * converts RPM to ticks per one hundred ms
     * @param speedRPM the speed in rpm
     * @return the amount of ticks from rpm
     */
    private double RPMToTicksPerOneHundredMS(double speedRPM) {
        return (speedRPM/minuteInHundredMs) * (TICKS_PER_REV/GEAR_RATIO);
    }

    /**
     * converts ticks per one hundred ms to rpm
     * @param ticksPerHundredMs amount of ticks per hundred ms
     * @return the amount of rpm from ticks
     */
    private double TicksPerOneHundredMSToRPM(double ticksPerHundredMs) {
        return ticksPerHundredMs * (GEAR_RATIO/TICKS_PER_REV) * minuteInHundredMs;
  }
}
