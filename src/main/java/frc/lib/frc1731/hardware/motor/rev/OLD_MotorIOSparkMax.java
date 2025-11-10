package frc.lib.frc1731.hardware.motor.rev;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.lib.frc1731.hardware.motor.PortConfig;

/**
 * Wrapper class for motors that use the Spark Max motor controller
 */
public class OLD_MotorIOSparkMax extends OLD_MotorIOSparkBase<SparkMax, SparkMaxConfig> {
    public OLD_MotorIOSparkMax(PortConfig config) {
        super(
            config.kBus,
            new SparkMax(config.kPort, MotorType.kBrushless), 
            new SparkMaxConfig(),
            config.kInverted
        );
    }
}