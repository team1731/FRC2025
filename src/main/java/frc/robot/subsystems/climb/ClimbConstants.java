package frc.robot.subsystems.climb;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

public class ClimbConstants {
    
    // climb can ID
    public final static int climbCanId = 31;
    public final static int climbCancoderDeviceId = 28; 

    // motor direction
    public final static InvertedValue climbMotorDirection = InvertedValue.Clockwise_Positive;
    public final static SensorDirectionValue climbCanConderDirection = SensorDirectionValue.Clockwise_Positive;

    // output
    //public final static double idleOutput = 0;

    // positions
    public final static double minClimbPosition = 0;
    public final static double maxClimbPosition = 0.7;
    public final static double climbHomePosition = 0;
    public final static double climbReadyPosition = 0.365;
    public final static double climbResetThreshold = .1; // indicates when climb is being reset and isClimbing should be set to false
    public final static double climbArmStowThreshold = .5; // indicates when arm should move out of the way
    
}
