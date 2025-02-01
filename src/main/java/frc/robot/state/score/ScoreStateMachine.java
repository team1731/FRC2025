package frc.robot.state.score;

import frc.robot.state.StateMachine;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

public class ScoreStateMachine extends StateMachine {
    private ElevatorSubsystem elevatorSubsystem;
    //private ArmSubsystem armSubsystem;

    private Object STATE_TRANSITION_TABLE[][] = {
        // CURRENT                           INPUT                                  OPERATION                    NEXT
        {ScoreState.HOME,                    ScoreInput.BEGIN,                      "raiseElevator",             ScoreState.RAISING},
        {ScoreState.RAISING,                 ScoreInput.ELEVATOR_THRESHOLD_MET,     "positionArmToScore",        ScoreState.POSITIONING},
        {ScoreState.POSITIONING,             ScoreInput.ARM_DONE,                   "scoreGamePiece",            ScoreState.SCORING},
        {ScoreState.SCORING,                 ScoreInput.PIECE_SCORED,               "moveArmHome",               ScoreState.ARM_RESETTING},
        {ScoreState.ARM_RESETTING,           ScoreInput.ARM_DONE,                   "lowerElevator",             ScoreState.LOWERING},
        {ScoreState.LOWERING,                ScoreInput.ELEVATOR_DONE,              "doSafetyCheck",             ScoreState.CHECKING_SAFETY},
        {ScoreState.CHECKING_SAFETY,         ScoreInput.IS_SAFE,                    null,                        ScoreState.HOME}
        // TODO define state sequence for recovery
        // TODO define state sequence for retrieving from floor pickup
    };


    public ScoreStateMachine(ElevatorSubsystem elevatorSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;
        setStateTransitionTable(STATE_TRANSITION_TABLE);
        setCurrentState(ScoreState.HOME);
    }

    public void setScoreConditions(ScoreAction action, GamePiece piece) {
        // TODO define flexibility to handle different scoring behaviors and game pieces
    }

    /*
     * STATE OPERATION METHODS
     */

    public boolean isReady() {
        return currentState == ScoreState.HOME;
    }

    public boolean raiseElevator() {
        // TODO
        return true;
    }

    public boolean lowerElevator() {
        // TODO
        return true;
    }

    public boolean positionArmToScore() {
        // TODO
        return true;
    }

    public boolean moveArmHome() {
        // TODO
        return true;
    }

    public boolean scoreGamePiece() {
        // TODO
        return true;
    }

    public boolean doSafetyCheck() {
        // TODO
        return true;
    }

    /*
     * SAFETY AND RECOVERY METHODS
     */

    public void interrupt() {
        // TODO, stop and return to home position
    }

    public boolean isSafe() {
        // TODO, checks to see if the scoring system is in a safe/home position
        return true;
    }

    public void recover() {
        // TODO, detects component positions and carefully returns to a safe/home state
    }
}
