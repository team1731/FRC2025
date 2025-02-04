package frc.robot.state.score.constants;

public class ScorePositions {
    public double raiseElevatorPosition;
    public double raiseElevatorThreshold;
    public double lowerElevatorThreshold;
    public double armForwardPosition;
    public double armScoringPosition;

    // Constructor for SCORE CORAL sequences
    public ScorePositions(double raisePosition, double raiseThreshold, double lowerThreshold, double armPosition, double scorePosition) {
        raiseElevatorPosition = raisePosition;
        raiseElevatorThreshold = raiseThreshold;
        lowerElevatorThreshold = lowerThreshold;
        armForwardPosition = armPosition;
        armScoringPosition = scorePosition;
    }
}
