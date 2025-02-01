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
}
