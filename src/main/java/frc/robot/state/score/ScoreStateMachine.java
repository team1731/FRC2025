package frc.robot.state.score;

import frc.robot.state.Input;
import frc.robot.state.StateMachine;
import frc.robot.state.StateMachineCallback;
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
    private StateMachineCallback resetCallback = (Input input) -> {
        if(isResetting) {
            System.out.println("reset done" + input + " " + elevatorResetDone + " " + armResetDone);
            //ScoreInput scoreInput = (ScoreInput)input;
            //if(scoreInput == ScoreInput.ELEVATOR_DONE) elevatorResetDone = true;
            //if(scoreInput == ScoreInput.ARM_DONE) armResetDone = true;
            //if(elevatorResetDone && armResetDone) {
                setInput(ScoreInput.RESET_DONE);
            //}
        }
    };

    private Object STATE_TRANSITION_TABLE[][] = {
        // CURRENT                           INPUT                                  OPERATION                    NEXT
        {ScoreState.HOME,                    ScoreInput.BEGIN,                      "raiseElevator",             ScoreState.RAISING_ELEVATOR},
        {ScoreState.RAISING_ELEVATOR,        ScoreInput.ELEVATOR_DONE,              "moveArmForward",            ScoreState.MOVING_ARM_FORWARD},
        {ScoreState.MOVING_ARM_FORWARD,      ScoreInput.ARM_DONE,                   null,                        ScoreState.WAITING},
        {ScoreState.WAITING,                 ScoreInput.SCORE,                      "moveArmToScore",            ScoreState.SCORING},
        {ScoreState.SCORING,                 ScoreInput.ARM_DONE,                   "lowerElevator",             ScoreState.LOWERING},
        {ScoreState.LOWERING,               ScoreInput.ELEVATOR_THRESHOLD_MET,      "moveArmBack",               ScoreState.RESETTING},
        {ScoreState.RESETTING,               ScoreInput.ARM_DONE,                 null,                     ScoreState.HOME},
        {ScoreState.ABORTING,                ScoreInput.ARM_DONE,                 "doSafetyCheck",             ScoreState.HOME}
    };


    public ScoreStateMachine(ElevatorSubsystem elevatorSubsystem, ArmSubsystem armSubsystem, HandClamperSubsystem handSubsystem, HandIntakeSubsystem handIntakeSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.armSubsystem = armSubsystem;

        setStateTransitionTable(STATE_TRANSITION_TABLE);
        setCurrentState(ScoreState.HOME);
    }

    public void setScoreConditions(ScoreAction action) {
        // set score positions
        if(action == ScoreAction.CORAL_L1) scorePositions = ScorePositionConstants.L1CoralScorePositions;
        if(action == ScoreAction.CORAL_L2) scorePositions = ScorePositionConstants.L2CoralScorePositions;
        if(action == ScoreAction.CORAL_L3) scorePositions = ScorePositionConstants.L3CoralScorePositions;
        if(action == ScoreAction.CORAL_L4) scorePositions = ScorePositionConstants.L4CoralScorePositions;
    }

    /*
     * STATE OPERATION METHODS
     */

     public boolean raiseElevator(){
        elevatorSubsystem.moveElevator(scorePositions.raiseElevatorPosition, inputCallback);
        return true;
     }

     public boolean lowerElevator() {
        elevatorSubsystem.moveElevator(ElevatorConstants.elevatorHomePosition, inputCallback, scorePositions.lowerElevatorThreshold);
        return true;
     }

     public boolean moveArmForward(){
        armSubsystem.moveArm(scorePositions.armForwardPosition, inputCallback);
        return true;
     }

     public boolean moveArmBack(){
        isResetting = true;
        armSubsystem.moveArm(ArmConstants.armHomePosition, inputCallback);
        return true;
     }

     public boolean moveArmToScore(){
        armSubsystem.moveArm(scorePositions.armScoringPosition, inputCallback);
        return true;
     }

     public boolean stop() {
        armSubsystem.stopArm();
        elevatorSubsystem.stopElevator();
        return true;
     }

     public boolean reset() {
        isResetting = true;
        //elevatorSubsystem.moveElevator(ElevatorConstants.elevatorHomePosition, resetCallback);
        //armSubsystem.moveArm(ArmConstants.armHomePosition, resetCallback);
        return true;
     }

     public boolean doSafetyCheck() {
        if(isSafe()) {
            System.out.println("yes is safe");
            resetInternalState();
           // processComplete();
            return true;
        } else {
            System.out.println("no is not safe");
            recover();
            return false;
        }
    }

    public boolean resetInternalState() {
        scorePositions = null;
        isResetting = false;
        elevatorResetDone = false;
        armResetDone = false;
        return true;
    }

    public boolean isReady() {
        return currentState == ScoreState.HOME;
    }

    /*
     * SAFETY AND RECOVERY METHODS
     */

    public void endSequence() {
        System.out.println("ending sequence");
        if(currentState == ScoreState.WAITING) { // TODO need to check if close to end of arm movement?
            setInput(ScoreInput.SCORE);
        } else {
            stop();
            setCurrentState(ScoreState.ABORTING);
            reset();
        }
    }

    public boolean isSafe() {
        System.out.println("is elevator safe? " + elevatorSubsystem.getElevatorPosition());
        System.out.println("is arm safe? " + armSubsystem.getArmPosition());
        // if(elevatorSubsystem.isAtPosition(ElevatorConstants.elevatorHomePosition) &&
        //    armSubsystem.isAtPosition(ArmConstants.armHomePosition)) {
        //     return true;
        // } else {
        //     System.out.println("ScoreStateMachine: Not in a safe position! One or both of elevator/arm subsystems are not at home");
        // 
        //    return false;
        return true;
        // }
    }

    public void recover() {
        // TODO need to implement
        // May be able to detect positions with sensors (if available) and recover automatically
        // Or, may need to hand over control to the operator to manually set to a safe position
    }
}
