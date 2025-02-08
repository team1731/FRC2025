package frc.robot.state.score.sequence;

import frc.robot.state.score.GamePiece;
import frc.robot.state.score.Level;
import frc.robot.state.score.ScoreStateMachine;
import frc.robot.state.score.Action;
import frc.robot.state.score.constants.PositionConstants;
import frc.robot.state.score.constants.ScorePositions;
import frc.robot.state.score.constants.TransitionConstants;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.hand.HandClamperSubsystem;
import frc.robot.subsystems.hand.HandIntakeSubsystem;

public class SequenceFactory {
    private static ScoreStateMachine scoreStateMachine;
    private static Level levelSelection = Level.L2; // L2 is default
    private static Action actionSelection;
    private static GamePiece pieceSelection = GamePiece.CORAL; // coral is default

    public static void setOperatorLevelSelection(Level level) {
        levelSelection = level;
    }

    public static void setOperatorPieceSelection(GamePiece piece) {
        pieceSelection = piece;
    }

    public static GamePiece getOperatorPieceSelection() {
        return pieceSelection;
    }

    public static void setDriverActionSelection(Action action) {
        actionSelection = action;
    }

    public static ScoreStateMachine getScoreStateMachine(ElevatorSubsystem elevatorSubsystem, ArmSubsystem armSubsystem, HandClamperSubsystem clamperSubsystem, HandIntakeSubsystem intakeSubsystem) {
        scoreStateMachine = new ScoreStateMachine(elevatorSubsystem, armSubsystem, clamperSubsystem, intakeSubsystem);
        return scoreStateMachine;
    }

    public static Sequence getSequence() {
        /*
         * CORAL SEQUENCES
         */
        // Coral intake
        if(pieceSelection == GamePiece.CORAL && levelSelection == Level.L1 && actionSelection == Action.INTAKE) return Sequence.INTAKE_CORAL_FLOOR;
        if(pieceSelection == GamePiece.CORAL && levelSelection == Level.L2 && actionSelection == Action.INTAKE) return Sequence.INTAKE_CORAL_FLOOR_UPRIGHT;
        if(pieceSelection == GamePiece.CORAL && levelSelection == Level.L3 && actionSelection == Action.INTAKE) return Sequence.INTAKE_CORAL_FEEDER;
        // Coral score
        if(pieceSelection == GamePiece.CORAL && levelSelection == Level.L1 && actionSelection == Action.SCORE) return Sequence.SCORE_CORAL_L1;
        if(pieceSelection == GamePiece.CORAL && levelSelection == Level.L2 && actionSelection == Action.SCORE) return Sequence.SCORE_CORAL_L2;
        if(pieceSelection == GamePiece.CORAL && levelSelection == Level.L3 && actionSelection == Action.SCORE) return Sequence.SCORE_CORAL_L3;
        if(pieceSelection == GamePiece.CORAL && levelSelection == Level.L4 && actionSelection == Action.SCORE) return Sequence.SCORE_CORAL_L4;

        /*
         * ALGAE SEQUENCES
         */
        // Algae intake
        if(pieceSelection == GamePiece.ALGAE && levelSelection == Level.L1 && actionSelection == Action.INTAKE) return Sequence.INTAKE_ALGAE_FLOOR;
        if(pieceSelection == GamePiece.ALGAE && levelSelection == Level.L2 && actionSelection == Action.INTAKE) return Sequence.INTAKE_ALGAE_L2;
        if(pieceSelection == GamePiece.ALGAE && levelSelection == Level.L3 && actionSelection == Action.INTAKE) return Sequence.INTAKE_ALGAE_L3;
        // Algae score
        if(pieceSelection == GamePiece.ALGAE && levelSelection == Level.L1 && actionSelection == Action.SCORE) return Sequence.HANDOFF_ALGAE;
        if(pieceSelection == GamePiece.ALGAE && levelSelection == Level.L4 && actionSelection == Action.SCORE) return Sequence.SHOOT_ALGAE;

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
