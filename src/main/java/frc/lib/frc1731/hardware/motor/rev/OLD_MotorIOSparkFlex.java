package frc.lib.frc1731.hardware.motor.rev;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import frc.lib.frc1731.hardware.motor.PortConfig;

/**
 * Wrapper class for motors that use the Spark Flex motor controller
 */
public class OLD_MotorIOSparkFlex extends OLD_MotorIOSparkBase<SparkFlex, SparkFlexConfig> {
    public OLD_MotorIOSparkFlex(PortConfig config) {
        super(
            config.kBus,
            new SparkFlex(config.kPort, MotorType.kBrushless), 
            new SparkFlexConfig(),
            config.kInverted
        );
    }
}