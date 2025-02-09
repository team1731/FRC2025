package frc.robot.state.sequencer.sequence;

import frc.robot.state.sequencer.Action;
import frc.robot.state.sequencer.GamePiece;
import frc.robot.state.sequencer.Level;
import frc.robot.state.sequencer.ScoreStateMachine;
import frc.robot.state.sequencer.constants.TransitionConstants;
import frc.robot.state.sequencer.positions.PositionConstants;
import frc.robot.state.sequencer.positions.Positions;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.hand.HandClamperSubsystem;
import frc.robot.subsystems.hand.HandIntakeSubsystem;

public class SequenceFactory {
    private static ScoreStateMachine scoreStateMachine;
    private static Level levelSelection = Level.L2; // L2 is default
    private static Action actionSelection;
    private static GamePiece pieceSelection = GamePiece.CORAL; // coral is default


    public static Level getOperatorLevelSelection() {
        return levelSelection;
    }

    public static void setOperatorLevelSelection(Level level) {
        levelSelection = level;
    }

    public static void setOperatorPieceSelection(GamePiece piece) {
        pieceSelection = piece;
    }

    public static GamePiece getOperatorPieceSelection() {
        return pieceSelection;
    }

    public static Action getDriverActionSelection() {
        return actionSelection;
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
        //if(pieceSelection == GamePiece.CORAL && levelSelection == Level.L1 && actionSelection == Action.INTAKE) return Sequence.INTAKE_CORAL_FLOOR;
        if(pieceSelection == GamePiece.CORAL && levelSelection == Level.L1 && actionSelection == Action.INTAKE) return Sequence.INTAKE_CORAL_FLOOR_UPRIGHT;
        if(pieceSelection == GamePiece.CORAL && levelSelection == Level.L2 && actionSelection == Action.INTAKE) return Sequence.INTAKE_CORAL_FEEDER;
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

    public static Positions getPositions(Sequence sequence) {
        switch(sequence) {

            /*
             * CORAL POSITIONS
             */
            case INTAKE_CORAL_FEEDER: return PositionConstants.CORAL_INTAKE.FEEDER_PICKUP_ACCESSOR;
            case INTAKE_CORAL_FLOOR: return null; // TODO needs definition
            case INTAKE_CORAL_FLOOR_UPRIGHT: return PositionConstants.CORAL_INTAKE.FLOOR_UPRIGHT_PICKUP_ACCESSOR;
            case SCORE_CORAL_L1: return PositionConstants.CORAL_SCORE.L1_ACCESSOR;
            case SCORE_CORAL_L2: return PositionConstants.CORAL_SCORE.L2_ACCESSOR;
            case SCORE_CORAL_L3: return PositionConstants.CORAL_SCORE.L3_ACCESSOR;
            case SCORE_CORAL_L4: return PositionConstants.CORAL_SCORE.L4_ACCESSOR;

            /*
             * ALGAE POSITIONS
             */
            case INTAKE_ALGAE_L2: return PositionConstants.ALGAE_INTAKE.REEF_PICKUP_L2;
            case INTAKE_ALGAE_L3: return PositionConstants.ALGAE_INTAKE.REEF_PICKUP_L3;
            case INTAKE_ALGAE_FLOOR: return PositionConstants.ALGAE_INTAKE.FLOOR_PICKUP;
            case SHOOT_ALGAE: return PositionConstants.ALGAE_SCORE.SCORE_BARGE;
            case HANDOFF_ALGAE: return PositionConstants.ALGAE_SCORE.HANDOFF;
        
            default:
                return null;
        }
    }
}
