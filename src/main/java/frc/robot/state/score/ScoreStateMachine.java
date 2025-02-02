package frc.robot.state.score;

import frc.robot.state.Input;
import frc.robot.state.StateMachine;
import frc.robot.state.StateMachineCallback;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.hand.HandIntakeSubsystem;
import frc.robot.subsystems.hand.HandSubsystem;

public class ScoreStateMachine extends StateMachine {

    // subsystems
    private ElevatorSubsystem elevatorSubsystem;
    private ArmSubsystem armSubsystem;

    // score tracking
    private ScoreAction scoreAction;
    private GamePiece gamePiece;
    private ScorePositions scorePositions;

    // reset/abort tracking
    private boolean isResetting = false;
    private boolean elevatorResetDone = false;
    private boolean armResetDone = false;
    private StateMachineCallback resetCallback = (Input input) -> {
        if(isResetting) {
            ScoreInput scoreInput = (ScoreInput)input;
            if(scoreInput == ScoreInput.ELEVATOR_DONE) elevatorResetDone = true;
            if(scoreInput == ScoreInput.ARM_DONE) armResetDone = true;
            if(elevatorResetDone && armResetDone) {
                setInput(ScoreInput.RESET_DONE);
            }
        }
    };

    private Object STATE_TRANSITION_TABLE[][] = {
        // CURRENT                           INPUT                                  OPERATION                    NEXT
        {ScoreState.HOME,                    ScoreInput.BEGIN,                      "raiseElevator",             ScoreState.RAISING_ELEVATOR},
        {ScoreState.RAISING_ELEVATOR,        ScoreInput.ELEVATOR_DONE,              "moveArmForward",            ScoreState.MOVING_ARM_FORWARD},
        {ScoreState.MOVING_ARM_FORWARD,      ScoreInput.ARM_DONE,                   null,                        ScoreState.WAITING},
        {ScoreState.WAITING,                 ScoreInput.SCORE,                      "moveArmToScore",            ScoreState.SCORING},
        {ScoreState.SCORING,                 ScoreInput.ARM_DONE,                   "reset",                     ScoreState.RESETTING},
        {ScoreState.RESETTING,               ScoreInput.RESET_DONE,                 "doSafetyCheck",             ScoreState.CHECKING_SAFETY},
        {ScoreState.ABORTING,                ScoreInput.RESET_DONE,                 "doSafetyCheck",             ScoreState.CHECKING_SAFETY},
        {ScoreState.CHECKING_SAFETY,         ScoreInput.IS_SAFE,                    "resetInternalState",        ScoreState.HOME}
    };


    public ScoreStateMachine(ElevatorSubsystem elevatorSubsystem, ArmSubsystem armSubsystem, HandSubsystem handSubsystem, HandIntakeSubsystem handIntakeSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.armSubsystem = armSubsystem;

        setStateTransitionTable(STATE_TRANSITION_TABLE);
        setCurrentState(ScoreState.HOME);
    }

    public void setScoreConditions(ScoreAction action, GamePiece piece) {
        scoreAction = action;
        gamePiece = piece;
        scorePositions = ScorePositionFactory.getScorePositions(action);
    }

    /*
     * STATE OPERATION METHODS
     */

     public boolean raiseElevator(){
        elevatorSubsystem.moveElevator(scorePositions.raiseElevatorPosition, inputCallback);
        return true;
     }

     public boolean moveArmForward(){
        armSubsystem.moveArm(scorePositions.armForwardPosition, inputCallback);
        return true;
     }

     public boolean moveArmToScore(){
        armSubsystem.moveArm(scorePositions.armScoringPosition, inputCallback);
        return true;
     }

     public boolean reset() {
        elevatorSubsystem.moveElevator(ElevatorConstants.elevatorHomePosition, resetCallback);
        armSubsystem.moveArm(ArmConstants.armHomePosition, resetCallback);
        return true;
     }

     public boolean doSafetyCheck() {
        if(isSafe()) {
            setInput(ScoreInput.IS_SAFE);
            processComplete();
        } else {
            recover();
        }
        return true;
    }

    public boolean resetInternalState() {
        scoreAction = null;
        gamePiece = null;
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

    public void interrupt() {
        if(currentState == ScoreState.WAITING) {
            setInput(ScoreInput.SCORE);
        } else {
            setCurrentState(ScoreState.ABORTING);
            reset();
        }
    }

    public boolean isSafe() {
        // TODO, checks to see if the scoring system is in a safe/home position
        return true;
    }

    public void recover() {
        // TODO, detects component positions and carefully returns to a safe/home state
        // once determine is safe then re-run safety check method
    }
}
