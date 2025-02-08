package frc.robot.state.score.constants;

public class ScorePositions {
    public double raiseElevatorPosition;
    public double raiseElevatorThreshold;
    public double lowerElevatorThreshold;
    public double armForwardPosition;
    public double armScoringPosition;
    public double handClamperPosition;

    /*
     * !!!!!!!!!!!!!!!!!!!!!!
     * CORAL POSITION HELPERS
     * !!!!!!!!!!!!!!!!!!!!!!
     */

    public static ScorePositions getCoralFeederPickupPositions(double clamperPosition) {
        ScorePositions positions = new ScorePositions();
        positions.handClamperPosition = clamperPosition;
        return positions;
    }

    public static ScorePositions getCoralUprightFloorPickupPositions(double armPosition, double clamperPosition) {
        ScorePositions positions = new ScorePositions();
        positions.armForwardPosition = armPosition;
        positions.handClamperPosition = clamperPosition;
        return positions;
    }

    public static ScorePositions getCoralScorePositions(double raisePosition, double raiseThreshold, double lowerThreshold, double armPosition, double scorePosition) {
        ScorePositions positions = new ScorePositions();
        positions.raiseElevatorPosition = raisePosition;
        positions.raiseElevatorThreshold = raiseThreshold;
        positions.lowerElevatorThreshold = lowerThreshold;
        positions.armForwardPosition = armPosition;
        positions.armScoringPosition = scorePosition;
        return positions;
    }


    /*
     * !!!!!!!!!!!!!!!!!!!!!!
     * CORAL POSITION HELPERS
     * !!!!!!!!!!!!!!!!!!!!!!
     */

    public static ScorePositions getAlgaeReefPickupPositions(double raisePosition, double raiseThreshold, double lowerThreshold, double armPosition, double clamperPosition) {
        ScorePositions positions = new ScorePositions();
        positions.raiseElevatorPosition = raisePosition;
        positions.raiseElevatorThreshold = raiseThreshold;
        positions.lowerElevatorThreshold = lowerThreshold;
        positions.armForwardPosition = armPosition;
        positions.handClamperPosition = clamperPosition;
        return positions;
    }

    public static ScorePositions getAlgaeFloorPickupPositions(double armPosition, double clamperPosition) {
        ScorePositions positions = new ScorePositions();
        positions.armForwardPosition = armPosition;
        positions.handClamperPosition = clamperPosition;
        return positions;
    }

    public static ScorePositions getAlgaeScorePositions(double raisePosition, double raiseThreshold, double lowerThreshold, double armPosition) {
        ScorePositions positions = new ScorePositions();
        positions.raiseElevatorPosition = raisePosition;
        positions.raiseElevatorThreshold = raiseThreshold;
        positions.lowerElevatorThreshold = lowerThreshold;
        positions.armForwardPosition = armPosition;
        return positions;
    }

    public static ScorePositions getAlgaeHandoffPositions(double armPosition) {
        ScorePositions positions = new ScorePositions();
        positions.armForwardPosition = armPosition;
        return positions;
    }
}
