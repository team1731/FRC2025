package frc.robot.subsystems.hand;

import com.ctre.phoenix6.signals.InvertedValue;

public final class HandConstants {
    public final static int clamperCanId = 40;
    public final static int intakeCanId = 38;

    public final static double minClamperPosition = 0;
    public final static double maxClamperPosition = 3;

    public final static double intakeHoldOutput = 0.1;

    // Motor Direction
    public final static InvertedValue clamperMotorDirection = InvertedValue.CounterClockwise_Positive;
    public final static InvertedValue intakeMotorDirection = InvertedValue.CounterClockwise_Positive;
}
