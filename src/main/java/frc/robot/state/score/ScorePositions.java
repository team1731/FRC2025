package frc.robot.state.score;

public class ScorePositions {
    public double raiseElevatorPosition;
    public double raiseElevatorThreshold;
    public double armForwardPosition;
    public double armScoringPosition;

    public ScorePositions(double raisePosition, double raiseThreshold, double armPosition, double scorePosition) {
        raiseElevatorPosition = raisePosition;
        raiseElevatorThreshold = raiseThreshold;
        armForwardPosition = armPosition;
        armScoringPosition = scorePosition;
    }
}
