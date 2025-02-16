package frc.robot.state.sequencer;

import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.hand.HandClamperSubsystem;
import frc.robot.subsystems.hand.HandIntakeSubsystem;

public class SequenceManager {
    private static SequenceStateMachine stateMachine;
    private static Level levelSelection = Level.L2; // L2 is default
    private static Action actionSelection;
    private static GamePiece pieceSelection = GamePiece.CORAL; // coral is default


    public static Level getLevelSelection() {
        return levelSelection;
    }

    public static void setLevelSelection(Level level) {
        levelSelection = level;
        evaluateForMidstreamUpdate(); // check to see if requires a midstream update to a running sequence
    }

    public static GamePiece getGamePieceSelection() {
        return pieceSelection;
    }

    public static void setGamePieceSelection(GamePiece piece) {
        pieceSelection = piece;
    }

    public static boolean shouldDetectGamePiece() {
        return pieceSelection == GamePiece.CORAL;
    }

    public static Action getActionSelection() {
        return actionSelection;
    }

    public static void setActionSelection(Action action) {
        actionSelection = action;
    }

    public static SequenceStateMachine getStateMachine(ElevatorSubsystem elevatorSubsystem, ArmSubsystem armSubsystem, HandClamperSubsystem clamperSubsystem, HandIntakeSubsystem intakeSubsystem) {
        if(stateMachine == null) {
            stateMachine = new SequenceStateMachine(elevatorSubsystem, armSubsystem, clamperSubsystem, intakeSubsystem);
        }
        return stateMachine;
    }

    public static Sequence getSequence() {
        return SequenceFactory.getSequence(levelSelection, pieceSelection, actionSelection);
    }

    private static void evaluateForMidstreamUpdate() {
        boolean updateLevel = false;
        Sequence updatedSequence = getSequence();
        
        if(stateMachine != null && stateMachine.hasLoadedTransitions() && !stateMachine.isReady()) {
            // state machine has loaded and is running a sequence
            if(sequenceChangeAllowsMidstreamUpdate(stateMachine.getCurrentSequence(), updatedSequence)) {
                // level change is permitted
                updateLevel = true;
            }
        }

        if(updateLevel) { // pass the level change request to the state machine and let it handle the update
            System.out.println("SequenceManager: Level change initiated. Requesting a change from " + 
                stateMachine.getCurrentSequence() + " to " +
                updatedSequence);
            stateMachine.overwriteSequenceForLevelChange(updatedSequence);
            stateMachine.setInput(SequenceInput.LEVEL_CHANGED);
        }
    }

    private static boolean sequenceChangeAllowsMidstreamUpdate(Sequence currentSequence, Sequence newSequence) {
        switch (currentSequence) {
            case SCORE_CORAL_L2:
                return (newSequence == Sequence.SCORE_CORAL_L3 || newSequence == Sequence.SCORE_CORAL_L4);
            case SCORE_CORAL_L3:
                return (newSequence == Sequence.SCORE_CORAL_L2 || newSequence == Sequence.SCORE_CORAL_L4);
            case SCORE_CORAL_L4:
                return (newSequence == Sequence.SCORE_CORAL_L2 || newSequence == Sequence.SCORE_CORAL_L3);
            case INTAKE_ALGAE_L2:
                return (newSequence == Sequence.INTAKE_ALGAE_L3);
            case INTAKE_ALGAE_L3:
                return (newSequence == Sequence.INTAKE_ALGAE_L2);
            default:
                break;
        }
        return false;
    }
}
