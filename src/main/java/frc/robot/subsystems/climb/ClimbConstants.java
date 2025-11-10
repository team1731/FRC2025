package frc.robot.subsystems.climb;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.*;

import frc.lib.frc1731.PIDGains;
import frc.lib.frc1731.hardware.motor.PortConfig;
import frc.robot.Constants;

public class ClimbConstants {
    
    // climb can ID
    public final static int climbCanId = 31;
    public final static int climbCancoderDeviceId = 28; 
    public final static InvertedValue climbMotorDirection = InvertedValue.Clockwise_Positive;
    public final static PortConfig climbPortConfig = new PortConfig(Constants.CANBUS_NAME, climbCanId, climbMotorDirection != InvertedValue.Clockwise_Positive);

    // Climb config
    public final static SensorDirectionValue climbCanConderDirection = SensorDirectionValue.Clockwise_Positive;

    public final static CANcoderConfiguration cancoderConfig = new CANcoderConfiguration().withMagnetSensor(
        new MagnetSensorConfigs()
            .withMagnetOffset(-0.315185546875)
            .withSensorDirection(ClimbConstants.climbCanConderDirection)
            .withAbsoluteSensorDiscontinuityPoint(0.900146484375)
    );

    public final static FeedbackConfigs feedbackConfigs = new FeedbackConfigs()
        .withFeedbackRemoteSensorID(climbCancoderDeviceId)
        .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
        .withRotorToSensorRatio(640d)
        .withSensorToMechanismRatio(1d)
    ;

    public final static PIDGains climbPIDGains = new PIDGains()
        // new PIDGains(240, 0, 0.0078125, 0.009375, 0.02); // kP, kI, kD, kV, kS
        .setPID(240d, 0d, 0.0078125d)
        .setV(0.009375d)
        .setS(0.02);
    
    // output
    //public final static double idleOutput = 0;

    // positions
    public final static double minClimbPosition = 0;
    public final static double maxClimbPosition = 0.7;
    public final static double climbHomePosition = 0;
    public final static double climbReadyPosition = 0.365;
    public final static double climbStowPosition = 0.47;
    public final static double climbResetThreshold = .1; // indicates when climb is being reset and isClimbing should be set to false
    public final static double climbArmStowThreshold = .5; // indicates when arm should move out of the way
    
    public final static double climbAtPositionThreshold = 2d;
}
