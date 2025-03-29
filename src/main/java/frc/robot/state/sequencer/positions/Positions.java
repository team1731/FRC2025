package frc.robot.state.sequencer.positions;

public class Positions {
    public double raiseElevatorPosition;
    public double raiseElevatorThreshold;
    public double secondStageElevatorPosition; // used when sequence includes multiple elevator movements before going home
    public double lowerElevatorThreshold;
    public double firstStageArmPosition;
    public double firstStageArmThreshold;
    public double secondStageArmPosition; // used when sequence includes multiple arm movements before going home
    public double clamperIntakePosition;
    public double clamperHoldPosition;
    public double clamperOpenPosition;
    public double clamperJigglePosition;
}
