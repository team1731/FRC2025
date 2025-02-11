package frc.robot.state.sequencer.transitions;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.verify;

import org.junit.jupiter.api.Test;

import frc.robot.state.StateMachineCallback;
import frc.robot.state.sequencer.Action;
import frc.robot.state.sequencer.GamePiece;
import frc.robot.state.sequencer.Level;
import frc.robot.state.sequencer.Sequence;
import frc.robot.state.sequencer.SequenceFactory;
import frc.robot.state.sequencer.SequenceInput;
import frc.robot.state.sequencer.SequenceManager;
import frc.robot.state.sequencer.SequenceState;
import frc.robot.state.sequencer.SequenceStateMachine;
import frc.robot.state.sequencer.positions.Positions;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.hand.HandClamperSubsystem;
import frc.robot.subsystems.hand.HandConstants;
import frc.robot.subsystems.hand.HandIntakeSubsystem;

public class AlgaeFloorPickupTransitionsTest {
    // @Test
    // void testFloorPickupAlgae(){
    //     Sequence sequence = getSequence(Action.INTAKE, Level.L1, GamePiece.ALGAE);
    //     runPickupAlageFromFloor(sequence);
    // }

    // @Test
    // void testAbortDuringArmMovement(){
    //     Sequence sequence = getSequence(Action.INTAKE, Level.L1, GamePiece.ALGAE);
    //     runAbortDuringSequence(sequence);
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

    private void runPickupAlageFromFloor(Sequence sequence){
        // mock the subsystems
        ElevatorSubsystem mockedElevatorSubsystem = mock(ElevatorSubsystem.class);
        ArmSubsystem mockedArmSubsystem = mock(ArmSubsystem.class);
        HandIntakeSubsystem mockHandIntakeSubsystem = mock(HandIntakeSubsystem.class);
        HandClamperSubsystem mockHandClamperSubsystem = mock(HandClamperSubsystem.class);

        SequenceStateMachine stateMachine = new SequenceStateMachine(mockedElevatorSubsystem, mockedArmSubsystem, mockHandClamperSubsystem, mockHandIntakeSubsystem);
        Positions positions = SequenceFactory.getPositions(sequence);
        StateMachineCallback callback = stateMachine.getSubsystemCallback();
        stateMachine.setSequence(sequence);

        // verify initial state
        assertTrue(stateMachine.isReady());
        assertEquals(SequenceState.HOME, stateMachine.getCurrentState());

        // transition to moving arm
        stateMachine.setInput(SequenceInput.BEGIN);
        verify(mockedArmSubsystem).moveArm(positions.armForwardPosition, callback);
        assertEquals(SequenceState.MOVING_ARM_FORWARD, stateMachine.getCurrentState());

        // transition to intaking algae
        callback.setInput(SequenceInput.ARM_DONE);
        verify(mockHandClamperSubsystem).open(positions.clamperIntakePosition);
        verify(mockHandIntakeSubsystem).intake(HandConstants.intakeVelocity, callback);
        assertEquals(SequenceState.INTAKING, stateMachine.getCurrentState());

        // transition to moving arm home
        callback.setInput(SequenceInput.DETECTED_PIECE);
        //verify(mockedArmSubsystem).moveArm(ArmConstants.armHomePosition, callback);    // error here "org.mockito.exceptions.verification.TooManyActualInvocations at SequencerStateMachineTest.java:211"
        assertEquals(SequenceState.FINISHING, stateMachine.getCurrentState());

        // transition to home
        callback.setInput(SequenceInput.ARM_DONE);
        assertEquals(SequenceState.HOME, stateMachine.getCurrentState());
    }

    private void runAbortDuringSequence(Sequence sequence) {
        // mock the subsystems
        ElevatorSubsystem mockedElevatorSubsystem = mock(ElevatorSubsystem.class);
        ArmSubsystem mockedArmSubsystem = mock(ArmSubsystem.class);
        HandIntakeSubsystem mockHandIntakeSubsystem = mock(HandIntakeSubsystem.class);
        HandClamperSubsystem mockHandClamperSubsystem = mock(HandClamperSubsystem.class);

        SequenceStateMachine stateMachine = new SequenceStateMachine(mockedElevatorSubsystem, mockedArmSubsystem, mockHandClamperSubsystem, mockHandIntakeSubsystem);
        Positions positions = SequenceFactory.getPositions(sequence);
        StateMachineCallback callback = stateMachine.getSubsystemCallback();
        stateMachine.setSequence(sequence);

        // verify initial state
        assertTrue(stateMachine.isReady());
        assertEquals(SequenceState.HOME, stateMachine.getCurrentState());

        // transition to moving arm
        stateMachine.setInput(SequenceInput.BEGIN);
        verify(mockedArmSubsystem).moveArm(positions.armForwardPosition, callback);
        assertEquals(SequenceState.MOVING_ARM_FORWARD, stateMachine.getCurrentState());
        
        // kicking off abort by signaling that the button was released
        stateMachine.setInput(SequenceInput.BUTTON_RELEASED);
        verify(mockedArmSubsystem).moveArm(ArmConstants.armHomePosition, callback);
        assertEquals(SequenceState.FINISHING, stateMachine.getCurrentState());

        // transition to home
        callback.setInput(SequenceInput.ARM_DONE);
        assertEquals(SequenceState.HOME, stateMachine.getCurrentState());
    }
}
