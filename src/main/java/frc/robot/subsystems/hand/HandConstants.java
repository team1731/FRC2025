package frc.robot.subsystems.hand;

import com.ctre.phoenix6.signals.InvertedValue;

public final class HandConstants {
    public final static int clamperCanId = 40;
    public final static int clamperCancoderDeviceId = 16;
    public final static int intakeCanId = 38;

    public final static double minClamperPosition = 0;
    public final static double maxClamperPosition = 0.22;

    public final static double intakeVelocity = 5000/60;
    public final static double intakeHoldOutput = 0.1;
    public final static double scoreAlgaeVelocity = 5000/60;
    public final static double intakeStoppedThreshold = 5;
    public final static double defaultReleaseRuntime = 1.0;

    // Motor Direction
    public final static InvertedValue clamperMotorDirection = InvertedValue.CounterClockwise_Positive;
    public final static InvertedValue intakeMotorDirection = InvertedValue.CounterClockwise_Positive;

    // Positions
    public final static double clamperHomePosition = 0;
    public final static double clamperCoralPosition = 0.018554;
    public final static double clamperAlgaePosition = 0.05;
    public final static double clamperReefIntakePosition = 0.22; // note: this should be wide for intaking algae from the reef

    // Position tolerance thresholds
    public final static double clamperPositionTolerance = 0.002;
}
