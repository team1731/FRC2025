package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.signals.InvertedValue;

public final class ElevatorConstants {
    public final static int elevatorCanId1 = 21;
    public final static int elevatorCanId2 = 22;

    public final static double MMVel = 2; // 5 rotations per second cruise
    public final static double MMAcc = 5; // Take approximately 0.5 seconds to reach max vel
    // Take approximately 0.2 seconds to reach max accel
    public final static double MMJerk = 50;

    // initialze PID controller and encoder objects
    public final static double kP = 60;
    public final static double kI = 0;
    public final static double kD = 0.1;
    public final static double kV = 0.12;
    public final static double kS = 0.25; // Approximately 0.25V to get the mechanism moving

    public final static double StM_Ratio = 1;

    // Motor Direction
    public final static InvertedValue elevatorDirection = InvertedValue.Clockwise_Positive; // or Clockwise_Positive

    // Thresholds
    public final static double minElevatorPosition = 0;
    public final static double maxElevatorPosition = 0;

}
