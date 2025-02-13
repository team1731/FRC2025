package frc.robot.subsystems.climb;

import com.ctre.phoenix6.signals.InvertedValue;

public class ClimbConstants {
    

    // climb can ID
    public final static int climbCanId = 31;
    public final static int climbCancoderDeviceId = 0; //TODO get this value

        //TODO apply proper constants for climb motor
    // motor direction
    public final static InvertedValue climbMotorDirection = InvertedValue.Clockwise_Positive;

    // output
    public final static double idleOutput = 0;

    // positions
    public final static double minClimbPosition = 0;
    public final static double maxClimbPosition = 0;
    public final static double climbHomePosition = 0;
    public final static double climbReadyPosition = 0;
    
}
