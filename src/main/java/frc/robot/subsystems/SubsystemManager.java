package frc.robot.subsystems;


import frc.robot.state.sequencer.Level;
import frc.robot.state.sequencer.positions.Positions;

public class SubsystemManager {
    private static boolean atScorePosition = false;

    public static Positions scorePosition;

    public static Level scoreLevel = Level.L4;

    public static boolean isAtScorePosition() {
        return atScorePosition;
    }
    public static void setAtScorePosition(boolean atScorePosition, Positions targetPosition, Level targetLevel) {
        SubsystemManager.atScorePosition = atScorePosition;
        scorePosition = targetPosition;
        scoreLevel = targetLevel;
    }

    public static Positions getScorePosition() {
        return scorePosition;
    }
    public static Level getScoreLevel() {
        return scoreLevel;
    }
}
