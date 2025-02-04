package frc.robot.state.score.constants;

import frc.robot.state.score.ScoreInput;
import frc.robot.state.score.ScoreState;

public final class TransitionConstants {
    public static final Object SCORE_CORAL_TRANSITION_TABLE[][] = {
        // CURRENT                           INPUT                                  OPERATION                    NEXT
        {ScoreState.HOME,                    ScoreInput.BEGIN,                      "raiseElevator",             ScoreState.RAISING_ELEVATOR},
        {ScoreState.RAISING_ELEVATOR,        ScoreInput.ELEVATOR_THRESHOLD_MET,     "moveArmForward",            ScoreState.MOVING_ARM_FORWARD},
        {ScoreState.MOVING_ARM_FORWARD,      ScoreInput.ARM_DONE,                   null,                        ScoreState.WAITING},
        {ScoreState.WAITING,                 ScoreInput.SCORE,                      "moveArmToScore",            ScoreState.SCORING},
        {ScoreState.SCORING,                 ScoreInput.ARM_DONE,                   "moveElevatorHome",          ScoreState.LOWERING},
        {ScoreState.LOWERING,                ScoreInput.ELEVATOR_THRESHOLD_MET,     "moveArmHome",               ScoreState.FINISHING},
        {ScoreState.FINISHING,               ScoreInput.RESET_DONE,                 "doSafetyCheck",             ScoreState.HOME},
        {ScoreState.ABORTING,                ScoreInput.RESET_DONE,                 "doSafetyCheck",             ScoreState.HOME}
    };
}
