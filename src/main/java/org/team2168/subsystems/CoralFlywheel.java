package org.team2168.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import org.team2168.Constants.CANDevices;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.AlternateEncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.annotations.Log;

public class CoralFlywheel extends SubsystemBase {
    private static SparkMax flywheelMotor = new SparkMax(CANDevices.CORAL_FLYWHEEL, MotorType.kBrushless);
    private static RelativeEncoder flywheelEncoder = flywheelMotor.getAlternateEncoder();
    private static DigitalInput coralDetector = new DigitalInput(CANDevices.LINE_BREAK_SENSOR);

    private final double minuteInHundredMs = 600.0;
    private final double TICKS_PER_REV = 8192;
    private final double GEAR_RATIO = 10;
    private final int SMART_CURRENT_LIMIT = 40;
    private boolean isInverted = false;
    private IdleMode coast = IdleMode.kCoast;

    public CoralFlywheel() {
        final SparkMaxConfig motorConfigs = new SparkMaxConfig();
        // final EncoderConfig encoderConfig = new EncoderConfig();
        final AlternateEncoderConfig altEncoderConfig = new AlternateEncoderConfig();


        motorConfigs
            .idleMode(coast)
            .inverted(isInverted)
            .smartCurrentLimit(SMART_CURRENT_LIMIT);

        // motorConfigs.encoder
        //     .apply(encoderConfig);

        motorConfigs.alternateEncoder
            .apply(altEncoderConfig)
            .countsPerRevolution(4096)
            .setSparkMaxDataPortConfig();


        motorConfigs.signals.externalOrAltEncoderPosition(5);

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
     * @param speedRPM the speed in rotations
     * @return the amount of ticks from rpm
     */
    private double RPMToTicksPerOneHundredMS(double speedRPM) {
        return (speedRPM/minuteInHundredMs) * (TICKS_PER_REV/GEAR_RATIO);
    }

    /**
     * converts ticks per one hundred ms to rpm
     * @param ticksPerHundredMs amount of ticks per hundred ms
     * @return the amount of rotations from ticks
     */
    private double TicksPerOneHundredMSToRPM(double ticksPerHundredMs) {
        return ticksPerHundredMs * (GEAR_RATIO/TICKS_PER_REV) * minuteInHundredMs;
    }

    /**
     * checks if there is a coral in the flywheel
     * @return false if no coral, true if there is a coral
     */
    @Log(name = "Is coral present?")
        public boolean isCoralPresent() {
            return !coralDetector.get();
        }
    
    /**
     * gets the speed in rotations per minute
     * @return amount of rotations
     */
    @Log(name = "flywheel speed (rotations per minutes)", rowIndex = 3, columnIndex = 1)
        public double getCoralFlywheelSpeedRPM() {
            return flywheelEncoder.getVelocity();
        }

    /**
     * gets the speed in ticks per hundred ms
     * @return amount of ticks per hundred ms
     */
    @Log(name = "flywheel speed (ticks per hundred ms)", rowIndex = 3, columnIndex = 1)
        public double getCoralFlywheelSpeedTicks() {
            return RPMToTicksPerOneHundredMS(flywheelEncoder.getVelocity());
        }       

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
            SmartDashboard.putBoolean("linebreak sensor", isCoralPresent());
    }
}
