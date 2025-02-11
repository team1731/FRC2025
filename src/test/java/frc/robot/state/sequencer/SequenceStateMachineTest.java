package frc.robot.state.sequencer;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.verify;
import org.junit.jupiter.api.Test;

import frc.robot.state.StateMachineCallback;
import frc.robot.state.sequencer.positions.Positions;
import frc.robot.state.sequencer.Sequence;
import frc.robot.state.sequencer.SequenceFactory;
import frc.robot.state.sequencer.SequenceManager;
import frc.robot.state.sequencer.SequenceStateMachine;
import frc.robot.state.sequencer.Action;
import frc.robot.state.sequencer.Level;
import frc.robot.state.sequencer.GamePiece;
import frc.robot.state.sequencer.SequenceInput;
import frc.robot.state.sequencer.SequenceState;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.hand.HandIntakeSubsystem;
import frc.robot.subsystems.hand.HandClamperSubsystem;
import frc.robot.subsystems.hand.HandConstants;

public class SequenceStateMachineTest {

    // @Test 
    // void testScoreCoralL2() {
    //     Sequence sequence = getSequence(Action.SCORE, Level.L2, GamePiece.CORAL);
    //     runCoralScoringSequence(sequence);
    // }

    // @Test 
    // void testScoreCoralL3() {
    //     Sequence sequence = getSequence(Action.SCORE, Level.L3, GamePiece.CORAL);
    //     runCoralScoringSequence(sequence);
    // }

    // @Test 
    // void testScoreCoralL4() {
    //     Sequence sequence = getSequence(Action.SCORE, Level.L4, GamePiece.CORAL);
    //     runCoralScoringSequence(sequence);
    // }

    // @Test 
    // void testScoreAlgae() {
    //     Sequence sequence = getSequence(Action.SCORE, Level.L4, GamePiece.ALGAE);
    //     runAlgaeScoringSequence(sequence);
    // }

    /*
     * COMMON METHODS FOR RUNNING SEQUENCES
     */

    private Sequence getSequence(Action action, Level level, GamePiece gamePiece) {
        SequenceManager.setActionSelection(action);
        SequenceManager.setLevelSelection(level);
        SequenceManager.setGamePieceSelection(gamePiece);
        return SequenceManager.getSequence();
    }

    private void runAlgaeScoringSequence(Sequence sequence) {
        // mock the subsystems
        ElevatorSubsystem mockedElevatorSubsystem = mock(ElevatorSubsystem.class);
        ArmSubsystem mockedArmSubsystem = mock(ArmSubsystem.class);
        HandIntakeSubsystem mockHandIntakeSubsystem = mock(HandIntakeSubsystem.class);
        HandClamperSubsystem mockHandClamperSubsystem = mock(HandClamperSubsystem.class);

        // setup the state machine, sequence, and associated position values
        SequenceStateMachine stateMachine = new SequenceStateMachine(mockedElevatorSubsystem, mockedArmSubsystem, mockHandClamperSubsystem, mockHandIntakeSubsystem);
        Positions positions = SequenceFactory.getPositions(sequence);
        StateMachineCallback callback = stateMachine.getSubsystemCallback();
        stateMachine.setSequence(sequence);

        // verify initial state
        assertTrue(stateMachine.isReady());
        assertEquals(SequenceState.HOME, stateMachine.getCurrentState());

        // transition to raise the elevator
        stateMachine.setInput(SequenceInput.BEGIN);
        verify(mockedElevatorSubsystem).moveElevator(positions.raiseElevatorPosition, callback, positions.raiseElevatorThreshold);
        assertEquals(SequenceState.RAISING_ELEVATOR, stateMachine.getCurrentState());

        // transition to move the arm
        callback.setInput(SequenceInput.ELEVATOR_THRESHOLD_MET);
        verify(mockedArmSubsystem).moveArm(positions.armForwardPosition, callback);
        assertEquals(SequenceState.MOVING_ARM_FORWARD, stateMachine.getCurrentState());

        // transition to waiting
        callback.setInput(SequenceInput.ARM_DONE);
        assertEquals(SequenceState.SCORING, stateMachine.getCurrentState());

        // transition to lowering
        callback.setInput(SequenceInput.RELEASED_PIECE);
        verify(mockedElevatorSubsystem).moveElevator(ElevatorConstants.elevatorHomePosition, callback, positions.lowerElevatorThreshold);
        assertEquals(SequenceState.LOWERING, stateMachine.getCurrentState());

        // transition to finishing
        callback.setInput(SequenceInput.ELEVATOR_THRESHOLD_MET);
        // TODO check this once releasing piece is defined, this seems wrong
        verify(mockedArmSubsystem).moveArm(ArmConstants.armHomePosition, callback);
        assertEquals(SequenceState.FINISHING, stateMachine.getCurrentState());

        // transition back to home
        callback.setInput(SequenceInput.ELEVATOR_DONE);
        callback.setInput(SequenceInput.ARM_DONE);
        assertEquals(SequenceState.HOME, stateMachine.getCurrentState());
    }

    private void runCoralScoringSequence(Sequence sequence) {
        // mock the subsystems
        ElevatorSubsystem mockedElevatorSubsystem = mock(ElevatorSubsystem.class);
        ArmSubsystem mockedArmSubsystem = mock(ArmSubsystem.class);
        HandIntakeSubsystem mockHandIntakeSubsystem = mock(HandIntakeSubsystem.class);
        HandClamperSubsystem mockHandClamperSubsystem = mock(HandClamperSubsystem.class);
        

        // setup the state machine, sequence, and associated position values
        SequenceStateMachine stateMachine = new SequenceStateMachine(mockedElevatorSubsystem, mockedArmSubsystem, mockHandClamperSubsystem, mockHandIntakeSubsystem);
        Positions positions = SequenceFactory.getPositions(sequence);
        StateMachineCallback callback = stateMachine.getSubsystemCallback();
        stateMachine.setSequence(sequence);

        // verify initial state
        assertTrue(stateMachine.isReady());
        assertEquals(SequenceState.HOME, stateMachine.getCurrentState());

        // transition to raise the elevator
        stateMachine.setInput(SequenceInput.BEGIN);
        verify(mockedElevatorSubsystem).moveElevator(positions.raiseElevatorPosition, callback, positions.raiseElevatorThreshold);
        assertEquals(SequenceState.RAISING_ELEVATOR, stateMachine.getCurrentState());

        // transition to move the arm
        callback.setInput(SequenceInput.ELEVATOR_THRESHOLD_MET);
        verify(mockedArmSubsystem).moveArm(positions.armForwardPosition, callback);
        assertEquals(SequenceState.MOVING_ARM_FORWARD, stateMachine.getCurrentState());

        // transition to waiting
        callback.setInput(SequenceInput.ARM_DONE);
        assertEquals(SequenceState.WAITING, stateMachine.getCurrentState());

        // transition to scoring
        callback.setInput(SequenceInput.BUTTON_RELEASED);
        verify(mockedArmSubsystem).moveArm(positions.armScoringPosition, callback);
        assertEquals(SequenceState.SCORING, stateMachine.getCurrentState());

        // transition to lowering
        callback.setInput(SequenceInput.ARM_DONE);
        verify(mockedElevatorSubsystem).moveElevator(ElevatorConstants.elevatorHomePosition, callback, positions.lowerElevatorThreshold);
        assertEquals(SequenceState.LOWERING, stateMachine.getCurrentState());

        // transition to finishing
        callback.setInput(SequenceInput.ELEVATOR_THRESHOLD_MET);
        verify(mockedArmSubsystem).moveArm(ArmConstants.armHomePosition, callback);
        assertEquals(SequenceState.FINISHING, stateMachine.getCurrentState());

        // transition back to home
        callback.setInput(SequenceInput.ELEVATOR_DONE);
        callback.setInput(SequenceInput.ARM_DONE);
        assertEquals(SequenceState.HOME, stateMachine.getCurrentState());
    }
}