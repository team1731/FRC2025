package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.signals.InvertedValue;

public final class ElevatorConstants {
    public final static int elevatorCanId1 = 21;
    public final static int elevatorCanId2 = 22;

    // Motor Direction
    public final static InvertedValue elevatorDirection = InvertedValue.Clockwise_Positive; // or Clockwise_Positive

    //Outputs
    public final static double idleOutput = 0;

    // Positions
    public final static double elevatorHomePosition = 0;
    public final static double minElevatorPosition = 0;
    public final static double maxElevatorPosition = 0;

}
