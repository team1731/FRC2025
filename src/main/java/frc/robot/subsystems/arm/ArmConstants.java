package frc.robot.subsystems.arm;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.*;

import frc.lib.frc1731.PIDGains;
import frc.lib.frc1731.hardware.motor.PortConfig;

public final class ArmConstants {
    //Arm Can ID
    public static final String armCANBus = "canivore2";
    public final static int armCanId = 33;
    public final static InvertedValue armMotorDirection = InvertedValue.Clockwise_Positive;
    public final static int armCancoderDeviceId = 15; 
    public final static PortConfig armPortConfig = new PortConfig(armCANBus, armCanId, armMotorDirection != InvertedValue.Clockwise_Positive);
    public final static double armGearRationModifier = (9.0/800.0); // modifies arm ticks into values for absolute encoder

    // Configurations
    public static final CANcoderConfiguration armCANCoderConfig = new CANcoderConfiguration().withMagnetSensor(
        new MagnetSensorConfigs()
            .withMagnetOffset(-0.2138671875) //-0.216552734375
            .withAbsoluteSensorDiscontinuityPoint(0.5)
            .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
    );

    public static final FeedbackConfigs armFeedbackConfig = new FeedbackConfigs()
            .withSensorToMechanismRatio(1d)
            .withFeedbackRemoteSensorID(ArmConstants.armCancoderDeviceId)
            .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
            .withRotorToSensorRatio(1600d / 18d);
            
    public static final PIDGains armPIDGains = new PIDGains()
        .setP(90d)
        .setD(0.0099)
        .setS(0.02) // Approximately 0.25V to get the mechanism moving
        .setV(0.9);

    public static final double armCurrentLimit = 40d; // amps

    // Motor Direction

    // Motion Magic Config
    // Fast/Normal
    public final static double normalArmVelocity = 90 * armGearRationModifier;
    public final static double normalArmAcceleration = 250;// * armGearRationModifier;
    public final static double armJerk = 0;

    // Slow
    public final static double slowedArmVelocity = 35 * armGearRationModifier;
    public final static double slowedArmAcceleration = 125;// * armGearRationModifier;

    //Outputs
    public final static double idleOutput = 0;

    // Positions
    public final static double armHomePosition = 0;
    public final static double minArmPosition = -0.09;
    public final static double maxArmPosition = 0.37; // TODO are these correct? Seems like arm can't actually go fully 90 degrees
    public final static double stowArmPosition = 19; // TODO are these correct?
    public final static double halfedArmPosition = 18;

    public final static double willSmackReefThreshold = 0.20; // above this, arm will hit the reef if we are against it

    public final static double atPositionThreshold = 0.06;
}