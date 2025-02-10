package frc.robot.state.score;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.verify;
import org.junit.jupiter.api.Test;
import static org.mockito.Mockito.times;

import frc.robot.state.StateMachineCallback;
import frc.robot.state.sequencer.Action;
import frc.robot.state.sequencer.GamePiece;
import frc.robot.state.sequencer.SequencerInput;
import frc.robot.state.sequencer.SequencerState;
import frc.robot.state.sequencer.SequenceStateMachine;
import frc.robot.state.sequencer.Sequence;
import frc.robot.state.sequencer.SequenceFactory;
import frc.robot.state.sequencer.positions.Positions;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.hand.HandConstants;
import frc.robot.subsystems.hand.HandIntakeSubsystem;
import frc.robot.subsystems.hand.HandClamperSubsystem;
public class ScoreStateMachineTest {

    // @Test 
    // void testScoreCoralL1() {
    //     Action action = Action.SCORE_L1;
    //     GamePiece gamePiece = GamePiece.CORAL;
    //     Sequence sequence = SequenceFactory.getSequence(action, gamePiece);
    //     runCoralScoringSequence(sequence, action, gamePiece);
    // }

    // @Test 
    // void testScoreCoralL2() {
    //     Action action = Action.SCORE_L2;
    //     GamePiece gamePiece = GamePiece.CORAL;
    //     Sequence sequence = SequenceFactory.getSequence(action, gamePiece);
    //     runCoralScoringSequence(sequence, action, gamePiece);
    // }

    // @Test 
    // void testScoreCoralL3() {
    //     Action action = Action.SCORE_L3;
    //     GamePiece gamePiece = GamePiece.CORAL;
    //     Sequence sequence = SequenceFactory.getSequence(action, gamePiece);
    //     runCoralScoringSequence(sequence, action, gamePiece);
    // }

    // @Test 
    // void testScoreCoralL4() {
    //     Action action = Action.SCORE_L3;
    //     GamePiece gamePiece = GamePiece.CORAL;
    //     Sequence sequence = SequenceFactory.getSequence(action, gamePiece);
    //     runCoralScoringSequence(sequence, action, gamePiece);
    // }

    // @Test 
    // void testScoreAlgae() {
    //     Action action = Action.SCORE_BARGE;
    //     GamePiece gamePiece = GamePiece.ALGAE;
    //     Sequence sequence = SequenceFactory.getSequence(action, gamePiece);
    //     runAlgaeScoringSequence(sequence, action, gamePiece);
    // }

    /*
     * COMMON METHODS FOR RUNNING SEQUENCES
     */

    private void runAlgaeScoringSequence(Sequence sequence, Action action, GamePiece gamePiece) {
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
        assertEquals(SequencerState.HOME, stateMachine.getCurrentState());

        // transition to raise the elevator
        stateMachine.setInput(SequencerInput.BEGIN);
        verify(mockedElevatorSubsystem).moveElevator(positions.raiseElevatorPosition, callback);
        assertEquals(SequencerState.RAISING_ELEVATOR, stateMachine.getCurrentState());

        // transition to move the arm
        callback.setInput(SequencerInput.ELEVATOR_THRESHOLD_MET);
        verify(mockedArmSubsystem).moveArm(positions.armForwardPosition, callback);
        assertEquals(SequencerState.MOVING_ARM_FORWARD, stateMachine.getCurrentState());

        // transition to waiting
        callback.setInput(SequencerInput.ARM_DONE);
        assertEquals(SequencerState.WAITING, stateMachine.getCurrentState());

        // transition to scoring
        callback.setInput(SequencerInput.SCORE);
        // TODO fix this once releasing piece is defined in state machine
        //verify(mockedArmSubsystem).moveArm(positions.armScoringPosition, callback);
        assertEquals(SequencerState.SCORING, stateMachine.getCurrentState());

        // transition to lowering
        callback.setInput(SequencerInput.RELEASED_PIECE);
        verify(mockedElevatorSubsystem).moveElevator(ElevatorConstants.elevatorHomePosition, callback, positions.lowerElevatorThreshold);
        assertEquals(SequencerState.LOWERING, stateMachine.getCurrentState());

        // transition to finishing
        callback.setInput(SequencerInput.ELEVATOR_THRESHOLD_MET);
        // TODO check this once releasing piece is defined, this seems wrong
        verify(mockedArmSubsystem).moveArm(ArmConstants.armHomePosition, callback);
        assertEquals(SequencerState.FINISHING, stateMachine.getCurrentState());

        // transition back to home
        callback.setInput(SequencerInput.ELEVATOR_DONE);
        callback.setInput(SequencerInput.ARM_DONE);
        assertEquals(SequencerState.HOME, stateMachine.getCurrentState());
    }

    private void runCoralScoringSequence(Sequence sequence, Action action, GamePiece gamePiece) {
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
        assertEquals(SequencerState.HOME, stateMachine.getCurrentState());

        // transition to raise the elevator
        stateMachine.setInput(SequencerInput.BEGIN);
        verify(mockedElevatorSubsystem).moveElevator(positions.raiseElevatorPosition, callback);
        assertEquals(SequencerState.RAISING_ELEVATOR, stateMachine.getCurrentState());

        // transition to move the arm
        callback.setInput(SequencerInput.ELEVATOR_THRESHOLD_MET);
        verify(mockedArmSubsystem).moveArm(positions.armForwardPosition, callback);
        assertEquals(SequencerState.MOVING_ARM_FORWARD, stateMachine.getCurrentState());

        // transition to waiting
        callback.setInput(SequencerInput.ARM_DONE);
        assertEquals(SequencerState.WAITING, stateMachine.getCurrentState());

        // transition to scoring
        callback.setInput(SequencerInput.SCORE);
        verify(mockedArmSubsystem).moveArm(positions.armScoringPosition, callback);
        assertEquals(SequencerState.SCORING, stateMachine.getCurrentState());

        // transition to lowering
        callback.setInput(SequencerInput.ARM_DONE);
        verify(mockedElevatorSubsystem).moveElevator(ElevatorConstants.elevatorHomePosition, callback, positions.lowerElevatorThreshold);
        assertEquals(SequencerState.LOWERING, stateMachine.getCurrentState());

        // transition to finishing
        callback.setInput(SequencerInput.ELEVATOR_THRESHOLD_MET);
        verify(mockedArmSubsystem).moveArm(ArmConstants.armHomePosition, callback);
        assertEquals(SequencerState.FINISHING, stateMachine.getCurrentState());

        // transition back to home
        callback.setInput(SequencerInput.ELEVATOR_DONE);
        callback.setInput(SequencerInput.ARM_DONE);
        assertEquals(SequencerState.HOME, stateMachine.getCurrentState());
    }
}
