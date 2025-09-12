package frc.lib.hardware.motor.rev;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.lib.hardware.motor.PortConfig;

/**
 * Wrapper class for motors that use the Spark Max motor controller
 */
public class MotorIOSparkMax extends MotorIOSparkBase<SparkMax, SparkMaxConfig> {
    public MotorIOSparkMax(PortConfig config) {
        super(
            config.kBus,
            new SparkMax(config.kPort, MotorType.kBrushless), 
            new SparkMaxConfig(),
            config.kInverted
        );
    }
}