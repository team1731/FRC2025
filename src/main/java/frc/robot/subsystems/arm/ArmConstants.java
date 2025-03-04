package frc.robot.subsystems.arm;

import com.ctre.phoenix6.signals.InvertedValue;

public final class ArmConstants {
    //Arm Can ID
    public final static int armCanId = 33;
    public final static int armCancoderDeviceId = 15; 
    public final static double armGearRationModifier = (9.0/800.0); // modifies arm ticks into values for absolute encoder

    // Motor Direction
    public final static InvertedValue armMotorDirection = InvertedValue.Clockwise_Positive;

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
    public final static double minArmPosition = 0;
    public final static double maxArmPosition = 0.37; // TODO are these correct? Seems like arm can't actually go fully 90 degrees
    public final static double stowArmPosition = 19; // TODO are these correct?
    public final static double halfedArmPosition = 18;
}