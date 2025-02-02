package frc.robot.state.score;

public class ScorePositionFactory {
    public static ScorePositions getScorePositions(ScoreAction action) {
        if(action == ScoreAction.CORAL_L1) return getL1CoralScorePositions();
        if(action == ScoreAction.CORAL_L2) return getL2CoralScorePositions();
        if(action == ScoreAction.CORAL_L3) return getL3CoralScorePositions();
        if(action == ScoreAction.CORAL_L4) return getL4CoralScorePositions();

        return null;
    }

    private static ScorePositions getL1CoralScorePositions() {
        return new ScorePositions(0, 0, 0, 0);
    }

    private static ScorePositions getL2CoralScorePositions() {
        return new ScorePositions(0, 0, 0, 0);
    }

    private static ScorePositions getL3CoralScorePositions() {
        return new ScorePositions(0, 0, 0, 0);
    }

    private static ScorePositions getL4CoralScorePositions() {
        return new ScorePositions(0, 0, 0, 0);
    }
}
