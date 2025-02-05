package frc.robot.state.score;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.verify;
import org.junit.jupiter.api.Test;
import frc.robot.state.StateMachineCallback;
import frc.robot.state.score.constants.ScorePositions;
import frc.robot.state.score.sequence.Sequence;
import frc.robot.state.score.sequence.SequenceFactory;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

public class ScoreStateMachineTest {
    @Test 
    void testScoreCoralL2() {
        // mock the subsystems
        ElevatorSubsystem mockedElevatorSubsystem = mock(ElevatorSubsystem.class);
        ArmSubsystem mockedArmSubsystem = mock(ArmSubsystem.class);

        // setup the state machine, sequence, and associated position values
        ScoreStateMachine stateMachine = new ScoreStateMachine(mockedElevatorSubsystem, mockedArmSubsystem);
        Sequence sequence = SequenceFactory.getSequence(Action.SCORE_L2, GamePiece.CORAL);
        ScorePositions positions = SequenceFactory.getPositions(sequence);
        StateMachineCallback callback = stateMachine.getSubsystemCallback();
        stateMachine.setSequence(sequence);

        // verify initial state
        assertEquals(Sequence.SCORE_CORAL_L2, sequence);
        assertTrue(stateMachine.isReady());
        assertEquals(ScoreState.HOME, stateMachine.getCurrentState());

        // transition to raise the elevator
        stateMachine.setInput(ScoreInput.BEGIN);
        verify(mockedElevatorSubsystem).moveElevator(positions.raiseElevatorPosition, callback);
        assertEquals(ScoreState.RAISING_ELEVATOR, stateMachine.getCurrentState());

        // transition to move the arm
        callback.setInput(ScoreInput.ELEVATOR_THRESHOLD_MET);
        verify(mockedArmSubsystem).moveArm(positions.armForwardPosition, callback);
        assertEquals(ScoreState.MOVING_ARM_FORWARD, stateMachine.getCurrentState());

        // transition to waiting
        callback.setInput(ScoreInput.ARM_DONE);
        assertEquals(ScoreState.WAITING, stateMachine.getCurrentState());

        // transition to scoring
        callback.setInput(ScoreInput.SCORE);
        verify(mockedArmSubsystem).moveArm(positions.armScoringPosition, callback);
        assertEquals(ScoreState.SCORING, stateMachine.getCurrentState());

        // transition to lowering
        callback.setInput(ScoreInput.ARM_DONE);
        verify(mockedElevatorSubsystem).moveElevator(ElevatorConstants.elevatorHomePosition, callback, positions.lowerElevatorThreshold);
        assertEquals(ScoreState.LOWERING, stateMachine.getCurrentState());

        // transition to finishing
        callback.setInput(ScoreInput.ELEVATOR_THRESHOLD_MET);
        verify(mockedArmSubsystem).moveArm(ArmConstants.armHomePosition, callback);
        assertEquals(ScoreState.FINISHING, stateMachine.getCurrentState());

        // transition back to home
        callback.setInput(ScoreInput.ELEVATOR_DONE);
        callback.setInput(ScoreInput.ARM_DONE);
        assertEquals(ScoreState.HOME, stateMachine.getCurrentState());
    }
}
