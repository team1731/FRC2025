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
        /*
         * CORAL SEQUENCES
         */
        if(gamePiece == GamePiece.CORAL && action == Action.INTAKE_FEEDER) return Sequence.INTAKE_CORAL_FEEDER;
        if(gamePiece == GamePiece.CORAL && action == Action.INTAKE_FLOOR) return Sequence.INTAKE_CORAL_FLOOR;
        if(gamePiece == GamePiece.CORAL && action == Action.INTAKE_FLOOR_UPRIGHT) return Sequence.INTAKE_CORAL_FLOOR_UPRIGHT;
        if(gamePiece == GamePiece.CORAL && action == Action.SCORE_L1) return Sequence.SCORE_CORAL_L1;
        if(gamePiece == GamePiece.CORAL && action == Action.SCORE_L2) return Sequence.SCORE_CORAL_L2;
        if(gamePiece == GamePiece.CORAL && action == Action.SCORE_L3) return Sequence.SCORE_CORAL_L3;
        if(gamePiece == GamePiece.CORAL && action == Action.SCORE_L4) return Sequence.SCORE_CORAL_L4;

        /*
         * ALGAE SEQUENCES
         */
        if(gamePiece == GamePiece.ALGAE && action == Action.INTAKE_L2) return Sequence.INTAKE_ALGAE_L2;
        if(gamePiece == GamePiece.ALGAE && action == Action.INTAKE_L3) return Sequence.INTAKE_ALGAE_L3;
        if(gamePiece == GamePiece.ALGAE && action == Action.INTAKE_FLOOR) return Sequence.INTAKE_ALGAE_FLOOR;
        if(gamePiece == GamePiece.ALGAE && action == Action.SCORE_BARGE) return Sequence.SHOOT_ALGAE;
        if(gamePiece == GamePiece.ALGAE && action == Action.HANDOFF) return Sequence.HANDOFF_ALGAE;

        return null;
    }

    public static Object[][] getTransitionTable(Sequence sequence) {
        switch(sequence) {

            /*
             * CORAL TRANSITIONS
             */
            case INTAKE_CORAL_FEEDER:
                return TransitionConstants.PICKUP_CORAL_FROM_FEEDER_TRANSITION_TABLE;
            case INTAKE_CORAL_FLOOR:
                return TransitionConstants.PICKUP_CORAL_FROM_FLOOR_TRANSITION_TABLE;
            case INTAKE_CORAL_FLOOR_UPRIGHT:
                return TransitionConstants.PICKUP_UPRIGHT_CORAL_FROM_FLOOR_TRANSITION_TABLE;
            case SCORE_CORAL_L1:
            case SCORE_CORAL_L2:
            case SCORE_CORAL_L3:
            case SCORE_CORAL_L4:
                return TransitionConstants.SCORE_CORAL_TRANSITION_TABLE;

            /*
             * ALGAE TRANSITIONS
             */
            case INTAKE_ALGAE_L2:
            case INTAKE_ALGAE_L3:
                return TransitionConstants.PICKUP_ALGAE_FROM_REEF_TRANSITION_TABLE;
            case INTAKE_ALGAE_FLOOR:
                return TransitionConstants.PICKUP_ALGAE_FROM_FLOOR_TRANSITION_TABLE;
            case SHOOT_ALGAE:
                return TransitionConstants.SCORE_ALGAE_TRANSITION_TABLE;
            case HANDOFF_ALGAE: 
                return TransitionConstants.HANDOFF_ALGAE_TRANSITION_TABLE;
        
            default:
                return null;
        }
    }

    public static ScorePositions getPositions(Sequence sequence) {
        switch(sequence) {

            /*
             * CORAL POSITIONS
             */
            case INTAKE_CORAL_FEEDER: return PositionConstants.CORAL_FEEDER_PICKUP_POSITIONS;
            case INTAKE_CORAL_FLOOR: return PositionConstants.CORAL_FLOOR_PICKUP_POSITIONS;
            case INTAKE_CORAL_FLOOR_UPRIGHT: return PositionConstants.CORAL_UPRIGHT_FLOOR_PICKUP_POSITIONS;
            case SCORE_CORAL_L1: return PositionConstants.CORAL_L1_SCORE_POSITIONS;
            case SCORE_CORAL_L2: return PositionConstants.CORAL_L2_SCORE_POSITIONS;
            case SCORE_CORAL_L3: return PositionConstants.CORAL_L3_SCORE_POSITIONS;
            case SCORE_CORAL_L4: return PositionConstants.CORAL_L4_SCORE_POSITIONS;

            /*
             * ALGAE POSITIONS
             */
            case INTAKE_ALGAE_L2: return PositionConstants.ALGAE_REEF_PICKUP_L2_POSITIONS;
            case INTAKE_ALGAE_L3: return PositionConstants.ALGAE_REEF_PICKUP_L3_POSITIONS;
            case INTAKE_ALGAE_FLOOR: return PositionConstants.ALGAE_FLOOR_PICKUP_POSITIONS;
            case SHOOT_ALGAE: return PositionConstants.ALGAE_SCORE_POSITIONS;
            case HANDOFF_ALGAE: return PositionConstants.ALGAE_HANDOFF_POSITIONS;
        
            default:
                return null;
        }
    }
}
