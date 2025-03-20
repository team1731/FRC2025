package frc.robot.state.sequencer;

import frc.robot.state.sequencer.positions.Positions;
import frc.robot.state.sequencer.positions.PositionsFactory;
import frc.robot.state.sequencer.transitions.AlgaeFloorPickupTransitions;
import frc.robot.state.sequencer.transitions.AlgaeHandoffTransitions;
import frc.robot.state.sequencer.transitions.AlgaeReefPickupTransitions;
import frc.robot.state.sequencer.transitions.AlgaeScoreBargeTransitions;
import frc.robot.state.sequencer.transitions.CoralFeederPickupTransitions;
import frc.robot.state.sequencer.transitions.CoralScoreL1Transitions;
import frc.robot.state.sequencer.transitions.CoralScoreTransitions;
import frc.robot.state.sequencer.transitions.ResetTransitions;
import frc.robot.state.sequencer.transitions.UnStuckElevator;

public class SequenceFactory {
    public static Sequence getSequence(Level levelSelection, GamePiece pieceSelection, Action actionSelection) { 
        /*
         * CORAL SEQUENCES
         */
        // Coral intake
        //Currently disabled may not need
        //if(pieceSelection == GamePiece.CORAL && levelSelection == Level.L1 && actionSelection == Action.INTAKE) return Sequence.INTAKE_CORAL_FLOOR;
        //if(pieceSelection == GamePiece.CORAL && levelSelection == Level.L1 && actionSelection == Action.INTAKE) return Sequence.INTAKE_CORAL_FLOOR_UPRIGHT;
        if(pieceSelection == GamePiece.CORAL && levelSelection == Level.L1 && actionSelection == Action.INTAKE) return Sequence.UNSTUCK_ELEVATOR;
        if(pieceSelection == GamePiece.CORAL && levelSelection == Level.L2 && actionSelection == Action.INTAKE) return Sequence.INTAKE_CORAL_FEEDER;
        if(pieceSelection == GamePiece.CORAL && levelSelection == Level.L3 && actionSelection == Action.INTAKE) return Sequence.INTAKE_CORAL_FEEDER;
        if(pieceSelection == GamePiece.CORAL && levelSelection == Level.L4 && actionSelection == Action.INTAKE) return Sequence.INTAKE_CORAL_FEEDER;
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
        if(pieceSelection == GamePiece.ALGAE && levelSelection == Level.L4 && actionSelection == Action.INTAKE) return Sequence.INTAKE_ALGAE_L3;
        // Algae score
        if(pieceSelection == GamePiece.ALGAE && levelSelection == Level.L1 && actionSelection == Action.SCORE) return Sequence.HANDOFF_ALGAE;
        if(pieceSelection == GamePiece.ALGAE && levelSelection == Level.L2 && actionSelection == Action.SCORE) return Sequence.SHOOT_ALGAE;
        if(pieceSelection == GamePiece.ALGAE && levelSelection == Level.L3 && actionSelection == Action.SCORE) return Sequence.SHOOT_ALGAE;
        if(pieceSelection == GamePiece.ALGAE && levelSelection == Level.L4 && actionSelection == Action.SCORE) return Sequence.SHOOT_ALGAE;

        return null;
    }

    public static Object[][] getTransitionTable(Sequence sequence) {
        switch(sequence) {
            case RESET:
                return ResetTransitions.getTransitionTable();
            /*
             * CORAL TRANSITIONS
             */
            case INTAKE_CORAL_FEEDER:
                return CoralFeederPickupTransitions.getTransitionTable();
            case INTAKE_CORAL_FLOOR:
                return null; // TODO not defined yet
            case INTAKE_CORAL_FLOOR_UPRIGHT:
                // Currently disabled, may not need
                return null; // return CoralUprightFloorPickupTransitions.getTransitionTable();
            case SCORE_CORAL_L1:
                return CoralScoreL1Transitions.getTransitionTable();
                //return CoralScoreL1AltTransitions.getTransitionTable();
            case SCORE_CORAL_L2:
            case SCORE_CORAL_L3:
            case SCORE_CORAL_L4:
                return CoralScoreTransitions.getTransitionTable();

            /*
             * ALGAE TRANSITIONS
             */
            case INTAKE_ALGAE_L2:
            case INTAKE_ALGAE_L3:
                return AlgaeReefPickupTransitions.getTransitionTable();
            case INTAKE_ALGAE_FLOOR:
                return AlgaeFloorPickupTransitions.getTransitionTable();
            case SHOOT_ALGAE:
                return AlgaeScoreBargeTransitions.getTransitionTable();
            case HANDOFF_ALGAE: 
                return AlgaeHandoffTransitions.getTransitionTable();

                
            case UNSTUCK_ELEVATOR:
                return UnStuckElevator.getTransitionTable();
        
            default:
                return null;
        }
    }

    public static Positions getPositions(Sequence sequence) {
        switch(sequence) {
            case RESET: return new Positions(); // empty positions, not needed for a reset
            /*
             * CORAL POSITIONS
             */
            case INTAKE_CORAL_FEEDER: return PositionsFactory.getCoralFeederPickupPositions();
            case INTAKE_CORAL_FLOOR: return null; // TODO needs definition
            //Currently disabled may not need
            //case INTAKE_CORAL_FLOOR_UPRIGHT: return PositionsFactory.getCoralUprightFloorPickupPositions();
            case SCORE_CORAL_L1: return PositionsFactory.getCoralScoreL1Positions();
            //case SCORE_CORAL_L1: return PositionsFactory.getCoralScoreL1AltPositions();
            case SCORE_CORAL_L2: return PositionsFactory.getCoralScoreL2Positions();
            case SCORE_CORAL_L3: return PositionsFactory.getCoralScoreL3Positions();
            case SCORE_CORAL_L4: return PositionsFactory.getCoralScoreL4Positions();

            /*
             * ALGAE POSITIONS
             */
            case INTAKE_ALGAE_L2: return PositionsFactory.getAlgaeReefL2PickupPositions();
            case INTAKE_ALGAE_L3: return PositionsFactory.getAlgaeReefL3PickupPositions();
            case INTAKE_ALGAE_FLOOR: return PositionsFactory.getAlgaeFloorPickupPositions();
            case SHOOT_ALGAE: return PositionsFactory.getAlgaeScoreBargePositions();
            case HANDOFF_ALGAE: return PositionsFactory.getAlgaeHandoffPositions();

            case UNSTUCK_ELEVATOR: return PositionsFactory.getUnStuckElevator();
        
            default:
                return null;
        }
    }
}
