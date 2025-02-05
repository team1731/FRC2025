package frc.robot.state.score;

import frc.robot.state.Input;
import frc.robot.state.StateMachine;
import frc.robot.state.StateMachineCallback;
import frc.robot.state.score.constants.ScorePositions;
import frc.robot.state.score.sequence.Sequence;
import frc.robot.state.score.sequence.SequenceFactory;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.hand.HandIntakeSubsystem;
import frc.robot.subsystems.hand.HandClamperSubsystem;

public class ScoreStateMachine extends StateMachine {
    // subsystems
    private ElevatorSubsystem elevatorSubsystem;
    private ArmSubsystem armSubsystem;

    // score tracking
    private ScorePositions scorePositions;

    // reset/abort tracking
    private boolean isResetting = false;
    private boolean elevatorResetDone = false;
    private boolean armResetDone = false;

    private StateMachineCallback subsystemCallback = (Input input) -> {
        if(isResetting) {
            ScoreInput scoreInput = (ScoreInput)input;
            if(scoreInput == ScoreInput.ELEVATOR_THRESHOLD_MET) setInput(input); // not home yet
            if(scoreInput == ScoreInput.ELEVATOR_DONE) elevatorResetDone = true;
            if(scoreInput == ScoreInput.ARM_DONE) armResetDone = true;
            if(elevatorResetDone && armResetDone) {
                setInput(ScoreInput.RESET_DONE);
            }
        } else {
            setInput(input);
        }
    };

    public ScoreStateMachine(ElevatorSubsystem elevatorSubsystem, ArmSubsystem armSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.armSubsystem = armSubsystem;
        setCurrentState(ScoreState.HOME);
    }

    public StateMachineCallback getSubsystemCallback() {
        return subsystemCallback;
    }

    /*
     * COMMAND INTERFACE
     */

    public boolean isReady() {
        return currentState == ScoreState.HOME;
    }

    public void setSequence(Sequence sequence) {
        // the sequence determines the choreographed movement of elevator/arm/hand
        setStateTransitionTable(SequenceFactory.getTransitionTable(sequence)); // state machine transitions
        scorePositions = SequenceFactory.getPositions(sequence); // position constants for subsystems
    }

    public void endSequence() {
        if(currentState == ScoreState.WAITING) { // TODO add support to check if *close* to end of movement
            setInput(ScoreInput.SCORE);
        } else {
            abort();
        }
    }

    /*
     * STATE OPERATION METHODS
     */

     public boolean raiseElevator() {
        elevatorSubsystem.moveElevator(scorePositions.raiseElevatorPosition, subsystemCallback);
        return true;
     }

     public boolean moveElevatorHome() {
        isResetting = true;
        elevatorSubsystem.moveElevator(ElevatorConstants.elevatorHomePosition, subsystemCallback, scorePositions.lowerElevatorThreshold);
        return true;
     }

     public boolean moveArmForward() {
        armSubsystem.moveArm(scorePositions.armForwardPosition, subsystemCallback);
        return true;
     }

     public boolean moveArmToScore() {
        armSubsystem.moveArm(scorePositions.armScoringPosition, subsystemCallback);
        return true;
     }

     public boolean moveArmHome() {
        armSubsystem.moveArm(ArmConstants.armHomePosition, subsystemCallback);
        return true;
     }

     public boolean prepareToIntake() {
        // TODO need to define
        return true;
     }

     public boolean shootToScore() {
        // TODO need to define
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
        isResetting = false;
        elevatorResetDone = false;
        armResetDone = false;
        return true;
    }

    private void abort() {
        setCurrentState(ScoreState.ABORTING);
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
