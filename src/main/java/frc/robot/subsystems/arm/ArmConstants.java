package frc.robot.subsystems.arm;

import com.ctre.phoenix6.signals.InvertedValue;

public final class ArmConstants {
    //Arm Can ID
    public final static int armCanId = 33;

    // Motor Direction
    public final static InvertedValue armMotorDirection = InvertedValue.Clockwise_Positive;

    // Motion Magic Config
    // Fast/Normal
    public final static double normalArmVelocity = 70;
    public final static double normalArmAcceleration = 250;
    public final static double armJerk = 0;
    // Slow
    public final static double slowedArmVelocity = 35;
    public final static double slowedArmAcceleration = 125;

    //Outputs
    public final static double idleOutput = 0;

    // Positions
    public final static double armHomePosition = 0;
    public final static double minArmPosition = 0;
    public final static double maxArmPosition = 20;
    public final static double stowArmPosition = 0; //TODO: (SF) get real value
}