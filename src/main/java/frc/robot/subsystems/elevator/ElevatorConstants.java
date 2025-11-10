package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.signals.InvertedValue;

import frc.lib.frc1731.PIDGains;
import frc.lib.frc1731.hardware.motor.PortConfig;

public final class ElevatorConstants {
    // Port Configs
    public final static String canBus = "canivore1";

    public final static int masterCanId1 = 21;
    public final static int followerCanId2 = 22;

    public final static InvertedValue elevatorMotor1Direction = InvertedValue.CounterClockwise_Positive;
    public final static InvertedValue elevatorMotor2Direction = InvertedValue.Clockwise_Positive;

    public final static PortConfig leadPortConfig = new PortConfig(
        ElevatorConstants.canBus, 
        ElevatorConstants.masterCanId1, 
        ElevatorConstants.elevatorMotor1Direction
    );

    public final static PortConfig followerPortConfig = new PortConfig(
        ElevatorConstants.canBus, 
        ElevatorConstants.followerCanId2, 
        ElevatorConstants.elevatorMotor2Direction
    );

    public final static PIDGains motionMagicGains = new PIDGains()
        .setP(4.9)
        .setD(0.0078125)
        .setS(0.02)
        .setV(0.14)
        .setG(0.1);

    public final static FeedbackConfigs feedbackConfigs = new FeedbackConfigs()
        .withSensorToMechanismRatio(1.0);

    // Mechanism Configs
    public final static double gearRatioModifier = (12.0/20);

    // Motion Magic Config
    public final static double normalElevatorVelocity = 95;
    public final static double normalElevatorAcceleration = 230;
    public final static double elevatorJerk = 0;

    public final static double slowedElevatorVelocity = 42;
    public final static double slowedElevatorAcceleration = 125;

    // Positions
    public final static double elevatorHomePosition = 0.0;
    public final static double minElevatorPosition = 0;
    public final static double maxElevatorPosition = 97 * gearRatioModifier;

    public final static double atPositionTolerance = 1.0;
}