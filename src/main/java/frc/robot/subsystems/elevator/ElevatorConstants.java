package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.signals.InvertedValue;

public final class ElevatorConstants {
    public final static int elevatorCanId1 = 21;
    public final static int elevatorCanId2 = 22;
    public final static double gearRatioModifier = (12.0/20);

    // Motor Direction
    public final static InvertedValue elevatorMotor1Direction = InvertedValue.CounterClockwise_Positive;
    public final static InvertedValue elevatorMotor2Direction = InvertedValue.Clockwise_Positive;

    // Motion Magic Config
    // Fast/Normal
    public final static double normalElevatorVelocity = 95;
    public final static double normalElevatorAcceleration = 230;
    public final static double elevatorJerk = 0;
    // Slow
    public final static double slowedElevatorVelocity = 42;
    public final static double slowedElevatorAcceleration = 125;

    //Outputs
    public final static double idleOutput = 0;

    // Positions
    public final static double elevatorHomePosition = 0.0;
    public final static double minElevatorPosition = 0;
    public final static double maxElevatorPosition = 97 * gearRatioModifier;

}