package frc.robot.state.sequencer;

import frc.robot.state.Input;
import frc.robot.state.StateMachine;
import frc.robot.state.StateMachineCallback;
import frc.robot.state.sequencer.positions.Positions;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.hand.HandIntakeSubsystem;
import frc.robot.subsystems.hand.HandClamperSubsystem;
import frc.robot.subsystems.hand.HandConstants;

public class SequenceStateMachine extends StateMachine {
    // subsystems
    private ElevatorSubsystem elevatorSubsystem;
    private ArmSubsystem armSubsystem;
    private HandIntakeSubsystem handIntakeSubsystem;
    private HandClamperSubsystem handClamperSubsystem;

    // sequence tracking
    private Sequence currentSequence;
    private GamePiece currentGamePiece;
    private Positions positions;

    // reset/abort tracking
    private boolean isResetting = false;
    private boolean elevatorResetDone = false;
    private boolean armResetDone = false;


    public SequenceStateMachine(ElevatorSubsystem elevatorSubsystem, ArmSubsystem armSubsystem, HandClamperSubsystem handClamperSubsystem, HandIntakeSubsystem handIntakeSubsystem) { //add hand here
        this.elevatorSubsystem = elevatorSubsystem;
        this.armSubsystem = armSubsystem;
        this.handClamperSubsystem = handClamperSubsystem;
        this.handIntakeSubsystem = handIntakeSubsystem;
        setCurrentState(SequencerState.HOME);
    }

    /*
     * COMMAND INTERFACE
     */

    public boolean isReady() {
        return currentState == SequencerState.HOME;
    }

    public void setSequence(Sequence sequence) {
        currentSequence = sequence;
        currentGamePiece = SequenceManager.getGamePieceSelection();
        // the sequence determines the choreographed movement of elevator/arm/hand
        setStateTransitionTable(SequenceFactory.getTransitionTable(sequence)); // state machine transitions
        positions = SequenceFactory.getPositions(sequence); // position constants for subsystems
    }


    /*
     * SUBSYSTEM INTERFACE
     */

    protected void handleSubsystemCallback(Input input) {
        if(isResetting) {
            SequencerInput sequenceInput = (SequencerInput)input;
            if(sequenceInput == SequencerInput.ELEVATOR_THRESHOLD_MET) setInput(input); // not home yet
            if(sequenceInput == SequencerInput.ELEVATOR_DONE) elevatorResetDone = true;
            if(sequenceInput == SequencerInput.ARM_DONE) armResetDone = true;
            if(elevatorResetDone && armResetDone) {
                setInput(SequencerInput.RESET_DONE);
            }
        } else {
            if(input == SequencerInput.RELEASED_PIECE) closeHand();
            if(input == SequencerInput.DETECTED_PIECE && currentGamePiece == GamePiece.CORAL) closeHand();
            if(currentSequence == Sequence.SCORE_CORAL_L2 && input == SequencerInput.ARM_DONE && currentState == SequencerState.SCORING) releasePiece();
            setInput(input);
        }
    }

    public StateMachineCallback getSubsystemCallback() {
        return subsystemCallback;
    }


    /*
     * STATE OPERATION METHODS
     */

    public boolean raiseElevator() {
        elevatorSubsystem.moveElevator(positions.raiseElevatorPosition, subsystemCallback, positions.raiseElevatorThreshold);
        return true;
    }

    public boolean moveElevatorHome() {
        isResetting = true;
        elevatorSubsystem.moveElevator(ElevatorConstants.elevatorHomePosition, subsystemCallback, positions.lowerElevatorThreshold);
        return true;
    }

    public boolean moveArmForward() {
        armSubsystem.moveArm(positions.armForwardPosition, subsystemCallback);
        return true;
    }

    public boolean moveArmToScore() {
        armSubsystem.moveArm(positions.armScoringPosition, subsystemCallback);
        return true;
    }

    public boolean moveArmHome() {
        armSubsystem.moveArm(ArmConstants.armHomePosition, subsystemCallback);
        return true;
    }

    public boolean closeHand() {
        handClamperSubsystem.close();
        return true;
     }

    public boolean prepareToIntake() {
        handClamperSubsystem.open(positions.handClamperPosition);
        handIntakeSubsystem.intake(HandConstants.intakeVelocity, subsystemCallback);
        return true;
    }

    public boolean releasePiece() {
        handIntakeSubsystem.stop();
        handIntakeSubsystem.release(HandConstants.intakeVelocity, HandConstants.defaultReleaseRuntime);
        setInput(SequencerInput.SCORED);
        return true;
    }

    public boolean shootToScore() {
        handIntakeSubsystem.release(HandConstants.scoreAlgaeVelocity, HandConstants.defaultReleaseRuntime, subsystemCallback);
        return true;
    }

    public boolean reset() {
        currentSequence = null;
        currentGamePiece = null;
        positions = null;
        isResetting = false;
        elevatorResetDone = false;
        armResetDone = false;
        return true;
    }
}
