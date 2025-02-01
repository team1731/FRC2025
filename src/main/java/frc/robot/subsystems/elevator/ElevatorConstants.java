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

    // public static final int ELEVATOR_CURRENT_LIMIT_A = 18;
    // public static final int ELEVATOR_HOLD_CURRENT_LIMIT_A = 5;
    // public static final int EJECT_CURRENT_LIMIT = 20;

    // public final static double elevatorSpeed = 0.25;

    // public final static double elevatorStartedVelocityThreshold = 1000;
    // public final static double elevatorHoldingVelocityThreshold = 60;

    // public static final double ELEVATOR_HOLD_POWER = 0.07;

    // // PID coefficients
    // public static final double kP = 5e-5;
    // public static final double kI = 1e-6;
    // public static final double kD = 0;
    // public static final double kIz = 0;
    // public static final double kFF = 0.000156;
    // public static final double kMaxOutput = 1;
    // public static final double kMinOutput = -1;
    // public static final double maxRPM = 5700;

    // // Smart Motion Coefficients
    // public static final double minVel = 500; // rpm
    // public static final double maxVel = 2000; // rpm
    // public static final double maxAcc = 1500;
    // public static final double allowedErr = 1;

    // public static final int smartMotionSlot = 0;

    // Motor Direction
    public final static InvertedValue elevatorDirection = InvertedValue.Clockwise_Positive; // or Clockwise_Positive

    // Positions
    public final static double elevatorHomePosition = 0;
    public final static double elevatorExtendedPosition = 75;
    public final static double elevatorPositionTolerance = 0.05;
    public static final double wristClearsPosition = 0;
    public static final double elevatorAmpPosition = 75;
    public static final double elevatorShooterAsIntakePosition = 20;
    public static final double elevatorAmpReversePosition = 10;
    public static double elevatorTrapPosition = 78.5;

}
