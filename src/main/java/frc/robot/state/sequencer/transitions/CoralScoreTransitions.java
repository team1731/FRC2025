package frc.robot.state.sequencer.transitions;

import frc.robot.state.sequencer.SequenceInput;
import frc.robot.state.sequencer.SequenceState;

public class CoralScoreTransitions {
    private static final Object transitionTable[][] = {
        // CURRENT                              INPUT                                     OPERATION                    NEXT
        {SequenceState.HOME,                    SequenceInput.BEGIN,                      "raiseElevator",             SequenceState.RAISING_ELEVATOR},
        {SequenceState.RAISING_ELEVATOR,        SequenceInput.ELEVATOR_THRESHOLD_MET,     "moveArmWithThreshold",      SequenceState.MOVING_ARM_FORWARD},
        {SequenceState.RAISING_ELEVATOR,        SequenceInput.AUTO_SCORE,                 null,                        SequenceState.AUTO_SCORE_WHEN_READY},
        {SequenceState.MOVING_ARM_FORWARD,      SequenceInput.ARM_THRESHOLD_MET,          "checkIfShouldScoreCoral",   SequenceState.WAITING}, // operation determines next state
        {SequenceState.MOVING_ARM_FORWARD,      SequenceInput.AUTO_SCORE,                 null,                        SequenceState.AUTO_SCORE_WHEN_READY},
        {SequenceState.WAITING,                 SequenceInput.BUTTON_RELEASED,            "moveArmToScoreCoral",       SequenceState.SCORING}, // driver initiated score
        //{SequenceState.WAITING,                 SequenceInput.SENSOR_SCORE,               "moveArmToScoreCoral",       SequenceState.SCORING}, // hand intake sensor initiated score
        {SequenceState.AUTO_SCORE_WHEN_READY,   SequenceInput.ELEVATOR_DONE,              "moveArmToScoreCoral",       SequenceState.SCORING}, // auto initiated score during elevator raise
        {SequenceState.AUTO_SCORE_WHEN_READY,   SequenceInput.ARM_DONE,                   "moveArmToScoreCoral",       SequenceState.SCORING}, // auto initiated score during arm forward
        {SequenceState.SCORING,                 SequenceInput.ARM_DONE,                   "moveElevatorHome",          SequenceState.LOWERING},
        {SequenceState.LOWERING,                SequenceInput.ELEVATOR_THRESHOLD_MET,     "moveArmHome",               SequenceState.FINISHING},
        {SequenceState.FINISHING,               SequenceInput.RESET_DONE,                 "resetState",                SequenceState.HOME},

        // Level change sequences
        {SequenceState.RAISING_ELEVATOR,        SequenceInput.LEVEL_CHANGED,              "updateElevator",            SequenceState.UPDATING_LEVEL},
        {SequenceState.MOVING_ARM_FORWARD,      SequenceInput.LEVEL_CHANGED,              "returnArmForUpdate",        SequenceState.MOVING_ARM_BACK},
        {SequenceState.WAITING,                 SequenceInput.LEVEL_CHANGED,              "returnArmForUpdate",        SequenceState.MOVING_ARM_BACK},
        {SequenceState.MOVING_ARM_BACK,         SequenceInput.ARM_DONE,                   "updateElevator",            SequenceState.UPDATING_LEVEL},
        {SequenceState.UPDATING_LEVEL,          SequenceInput.ELEVATOR_DONE,              "moveArmWithThreshold",      SequenceState.MOVING_ARM_FORWARD},
        
        // Abort sequences
        {SequenceState.RAISING_ELEVATOR,        SequenceInput.BUTTON_RELEASED,            "startReset",                SequenceState.FINISHING},
        {SequenceState.MOVING_ARM_FORWARD,      SequenceInput.BUTTON_RELEASED,            "startReset",                SequenceState.FINISHING}
    };
    

    public static Object [][] getTransitionTable() {
        return transitionTable;
    }
}