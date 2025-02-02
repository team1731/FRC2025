package frc.robot.state.score;

import frc.robot.state.StateMachine;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.hand.HandIntakeSubsystem;
import frc.robot.subsystems.hand.HandSubsystem;

public class ScoreStateMachine extends StateMachine {
    private ElevatorSubsystem elevatorSubsystem;
    private ArmSubsystem armSubsystem;
    private HandSubsystem handSubsystem;
    private HandIntakeSubsystem handIntakeSubsystem;

    private Object STATE_TRANSITION_TABLE[][] = {
        // CURRENT                           INPUT                                  OPERATION                    NEXT
        {ScoreState.HOME,                    ScoreInput.BEGIN,                      "raiseElevator",             ScoreState.RAISING_ELEVATOR},
        {ScoreState.RAISING_ELEVATOR,        ScoreInput.ELEVATOR_DONE,              "moveArmForward",            ScoreState.MOVING_ARM_FORWARD},
        {ScoreState.MOVING_ARM_FORWARD,      ScoreInput.ARM_DONE,                   null,                        ScoreState.WAITING},
        {ScoreState.WAITING,                 ScoreInput.SCORE,                      "moveArmToScore",            ScoreState.SCORING},
        {ScoreState.SCORING,                 ScoreInput.ARM_DONE,                   "lowerElevator",             ScoreState.LOWERING_ELEVATOR},
        {ScoreState.LOWERING_ELEVATOR,       ScoreInput.ELEVATOR_DONE,              "moveArmBack",               ScoreState.MOVING_ARM_BACK},
        {ScoreState.MOVING_ARM_BACK,         ScoreInput.ARM_DONE,                   "doSafetyCheck",             ScoreState.CHECKING_SAFETY},
        {ScoreState.CHECKING_SAFETY,         ScoreInput.IS_SAFE,                    null,                        ScoreState.HOME}
    };


    public ScoreStateMachine(ElevatorSubsystem elevatorSubsystem, ArmSubsystem armSubsystem, HandSubsystem handSubsystem, HandIntakeSubsystem handIntakeSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.armSubsystem = armSubsystem;
        this.handSubsystem = handSubsystem;
        this.handIntakeSubsystem = handIntakeSubsystem;

        setStateTransitionTable(STATE_TRANSITION_TABLE);
        setCurrentState(ScoreState.HOME);
    }

    public void setScoreConditions(ScoreAction action, GamePiece piece) {
        // TODO define flexibility to handle different scoring behaviors and game pieces
    }

    /*
     * STATE OPERATION METHODS
     */

     public boolean raiseElevator(){
        elevatorSubsystem.moveElevator(0, inputCallback);
        return true;
     }

     public boolean lowerElevator(){
        elevatorSubsystem.moveElevator(0, inputCallback);
        return true;
     }

     public boolean moveArmForward(){
        armSubsystem.moveArm(0, inputCallback);
        return true;
     }

     public boolean moveArmToScore(){
        armSubsystem.moveArm(0, inputCallback);
        return true;
     }

     public boolean moveArmBack(){
        armSubsystem.moveArm(0, inputCallback);
        return true;
     }

     public boolean doSafetyCheck() {
        // TODO, implement proper safety check
        setInput(ScoreInput.IS_SAFE);
        processComplete();
        return true;
    }

    public boolean isReady() {
        return currentState == ScoreState.HOME;
    }

    /*
     * SAFETY AND RECOVERY METHODS
     */

    public void interrupt() {
        if(currentState == ScoreState.WAITING) {
            setInput(ScoreInput.SCORE);
        } else {
            // TODO, stop and return to home position
        }
    }

    public boolean isSafe() {
        // TODO, checks to see if the scoring system is in a safe/home position
        return true;
    }

    public void recover() {
        // TODO, detects component positions and carefully returns to a safe/home state
    }
}
