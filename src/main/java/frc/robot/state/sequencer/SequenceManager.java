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

        // notify the state machine and it will handle it if applicable
        // if not applicable to the sequence/state it will be ignored
        if(stateMachine != null && stateMachine.hasLoadedTransitions()) {
            //stateMachine.setInput(SequenceInput.LEVEL_CHANGED);
        }
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
}
