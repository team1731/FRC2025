package frc.robot.subsystems.hand;

import com.ctre.phoenix6.signals.InvertedValue;

public final class HandConstants {
    public final static int handCanId = 21;
    public final static int handIntakeCanId = 21;

    public final static double minHandPosition = 0;
    public final static double maxHandPosition = 0;

    public final static double intakeHoldOutput = 0.1;

    // Motor Direction
    public final static InvertedValue handDirection = InvertedValue.Clockwise_Positive; // or Clockwise_Positive
}
