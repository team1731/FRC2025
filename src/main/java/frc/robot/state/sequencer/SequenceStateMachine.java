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

    // score tracking
    private Sequence currentSequence;
    private GamePiece currentGamePiece;
    private Positions scorePositions;

    // reset/abort tracking
    private boolean passedAbortPoint = false;
    private boolean isResetting = false;
    private boolean elevatorResetDone = false;
    private boolean armResetDone = false;

    private StateMachineCallback subsystemCallback = (Input input) -> {
        if(isResetting) {
            SequencerInput scoreInput = (SequencerInput)input;
            if(scoreInput == SequencerInput.ELEVATOR_THRESHOLD_MET) setInput(input); // not home yet
            if(scoreInput == SequencerInput.ELEVATOR_DONE) elevatorResetDone = true;
            if(scoreInput == SequencerInput.ARM_DONE) armResetDone = true;
            if(elevatorResetDone && armResetDone) {
                setInput(SequencerInput.RESET_DONE);
            }
        } else {
            if(input == SequencerInput.RELEASED_PIECE) closeHand();
            if(input == SequencerInput.DETECTED_PIECE && currentGamePiece == GamePiece.CORAL) closeHand();
            if(currentSequence == Sequence.SCORE_CORAL_L2 && input == SequencerInput.ARM_DONE && currentState == SequencerState.SCORING) releasePiece();
            setInput(input);
        }
    };

    public SequenceStateMachine(ElevatorSubsystem elevatorSubsystem, ArmSubsystem armSubsystem, HandClamperSubsystem handClamperSubsystem, HandIntakeSubsystem handIntakeSubsystem) { //add hand here
        this.elevatorSubsystem = elevatorSubsystem;
        this.armSubsystem = armSubsystem;
        this.handClamperSubsystem = handClamperSubsystem;
        this.handIntakeSubsystem = handIntakeSubsystem;
        setCurrentState(SequencerState.HOME);
    }

    public StateMachineCallback getSubsystemCallback() {
        return subsystemCallback;
    }

    /*
     * COMMAND INTERFACE
     */

    public boolean isReady() {
        return currentState == SequencerState.HOME;
    }

    public void setSequence(Sequence sequence) {
        currentSequence = sequence;
        // the sequence determines the choreographed movement of elevator/arm/hand
        setStateTransitionTable(SequenceFactory.getTransitionTable(sequence)); // state machine transitions
        scorePositions = SequenceFactory.getPositions(sequence); // position constants for subsystems
    }

    public void setGamePiece(GamePiece piece) {
        currentGamePiece = piece;
    }

    public void endSequence() {
        System.out.println("ScoreStateMachine: asked to end sequence - " + 
            "Current state: " + currentState + " " + 
            "Passed abort point " + passedAbortPoint + " " + 
            "Resetting " + isResetting);
        if(currentState == SequencerState.WAITING) { // TODO add support to check if *close* to end of movement
            setInput(SequencerInput.SCORE);
        } else if(!passedAbortPoint && !isResetting) {
            abort();
        }
    }

    /*
     * STATE OPERATION METHODS
     */

    public boolean raiseElevator() {
        elevatorSubsystem.moveElevator(scorePositions.raiseElevatorPosition, subsystemCallback, scorePositions.raiseElevatorThreshold);
        return true;
    }

    public boolean moveElevatorHome() {
        isResetting = true;
        passedAbortPoint = true;
        elevatorSubsystem.moveElevator(ElevatorConstants.elevatorHomePosition, subsystemCallback, scorePositions.lowerElevatorThreshold);
        return true;
    }

    public boolean moveArmForward() {
        armSubsystem.moveArm(scorePositions.armForwardPosition, subsystemCallback);
        return true;
    }

    public boolean moveArmToScore() {
        passedAbortPoint = true;
        armSubsystem.moveArm(scorePositions.armScoringPosition, subsystemCallback);
        return true;
    }

    public boolean moveArmHome() {
        passedAbortPoint = true;
        armSubsystem.moveArm(ArmConstants.armHomePosition, subsystemCallback);
        return true;
    }

    public boolean closeHand() {
        handClamperSubsystem.close();
        return true;
     }

    public boolean prepareToIntake() {
        handClamperSubsystem.open(scorePositions.handClamperPosition);
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
        passedAbortPoint = true;
        handIntakeSubsystem.release(HandConstants.scoreAlgaeVelocity, HandConstants.defaultReleaseRuntime, subsystemCallback);
        return true;
    }

    /*
     * RESET, SAFETY, AND RECOVERY METHODS
     */

    public boolean doSafetyCheck() {
        if(isSafe()) {
            resetInternalState();
            processComplete();
            return true;
        } else {
            recover();
            return false;
        }
    }

    public boolean isSafe() {
        // TODO for now just printing out info, but at some point probably want reliable safety checks
        System.out.println("ScoreStateMachine: checking safety -" + 
            " elevator pos: " + elevatorSubsystem.getElevatorPosition() +
            " arm pos: " + armSubsystem.getArmPosition());
        return true;
    }

    public boolean resetInternalState() {
        scorePositions = null;
        passedAbortPoint = false;
        isResetting = false;
        elevatorResetDone = false;
        armResetDone = false;
        return true;
    }

    private void abort() {
        setCurrentState(SequencerState.ABORTING);
        isResetting = true;
        // stop current movements
        armSubsystem.stopArm();
        elevatorSubsystem.stopElevator();
        // move back home
        armSubsystem.moveArm(ArmConstants.armHomePosition, subsystemCallback);
        elevatorSubsystem.moveElevator(ElevatorConstants.elevatorHomePosition, subsystemCallback);
     }

    public void recover() {
        // TODO need to implement
        // Move arm back into position using absolute encorder
        // Move elevator and clamper back into position using resistence to indicated home, then reset motor encoders
    }
}
