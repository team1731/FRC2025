package frc.robot.state.score.sequence;

import frc.robot.state.score.GamePiece;
import frc.robot.state.score.Action;
import frc.robot.state.score.constants.PositionConstants;
import frc.robot.state.score.constants.ScorePositions;
import frc.robot.state.score.constants.TransitionConstants;

public class SequenceFactory {
    private static Action operatorActionSelection;
    private static GamePiece operatorPieceSelection;

    public static void setOperatorActionSelection(Action action) {
        operatorActionSelection = action;
    }

    public static void setOperatorPieceSelection(GamePiece piece) {
        operatorPieceSelection = piece;
    }

    public static Sequence getSequence() {
        return getSequence(operatorActionSelection, operatorPieceSelection);
    }

    public static Sequence getSequence(Action action, GamePiece gamePiece) {
        if(gamePiece == GamePiece.CORAL && action == Action.SCORE_L1) return Sequence.SCORE_CORAL_L1;
        if(gamePiece == GamePiece.CORAL && action == Action.SCORE_L2) return Sequence.SCORE_CORAL_L2;
        if(gamePiece == GamePiece.CORAL && action == Action.SCORE_L3) return Sequence.SCORE_CORAL_L3;
        if(gamePiece == GamePiece.CORAL && action == Action.SCORE_L4) return Sequence.SCORE_CORAL_L4;

        return null;
    }

    public static Object[][] getTransitionTable(Sequence sequence) {
        switch (sequence) {
            case SCORE_CORAL_L1:
            case SCORE_CORAL_L2:
            case SCORE_CORAL_L3:
            case SCORE_CORAL_L4:
                return TransitionConstants.SCORE_CORAL_TRANSITION_TABLE;
        
            default:
                return null;
        }
    }

    public static ScorePositions getPositions(Sequence sequence) {
        switch (sequence) {
            case SCORE_CORAL_L1: return PositionConstants.CORAL_L1_POSITIONS;
            case SCORE_CORAL_L2: return PositionConstants.CORAL_L2_POSITIONS;
            case SCORE_CORAL_L3: return PositionConstants.CORAL_L3_POSITIONS;
            case SCORE_CORAL_L4: return PositionConstants.CORAL_L4_POSITIONS;
        
            default:
                return null;
        }
    }
}
