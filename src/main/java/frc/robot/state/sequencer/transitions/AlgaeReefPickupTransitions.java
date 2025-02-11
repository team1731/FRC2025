package frc.robot.state.sequencer.transitions;

import frc.robot.state.sequencer.SequenceInput;
import frc.robot.state.sequencer.SequenceState;

public class AlgaeReefPickupTransitions {
    private static final Object transitionTable[][] = {
        // CURRENT                              INPUT                                     OPERATION                    NEXT
        {SequenceState.HOME,                    SequenceInput.BEGIN,                      "raiseElevatorNoThreshold",  SequenceState.RAISING_ELEVATOR},
        {SequenceState.RAISING_ELEVATOR,        SequenceInput.ELEVATOR_DONE,              "prepareToIntake",           SequenceState.INTAKING},
        {SequenceState.INTAKING,                SequenceInput.BUTTON_RELEASED,            "elevatorSecondStage",       SequenceState.UPDATING_ELEVATOR},
        {SequenceState.UPDATING_ELEVATOR,       SequenceInput.ELEVATOR_DONE,              "holdAndLower",              SequenceState.LOWERING},
        {SequenceState.LOWERING,                SequenceInput.ELEVATOR_DONE,              "resetState",                SequenceState.HOME},

        // Level change sequences
        // TODO implement

        // Abort sequences
        {SequenceState.RAISING_ELEVATOR,        SequenceInput.BUTTON_RELEASED,            "startReset",               SequenceState.LOWERING}
    };



    /*
    private static final Object transitionTable[][] = {
        // CURRENT                              INPUT                                     OPERATION                    NEXT
        {SequenceState.HOME,                    SequenceInput.BEGIN,                      "raiseElevator",             SequenceState.RAISING_ELEVATOR},
        {SequenceState.RAISING_ELEVATOR,        SequenceInput.ELEVATOR_THRESHOLD_MET,     "moveArmForward",            SequenceState.MOVING_ARM_FORWARD},
        {SequenceState.MOVING_ARM_FORWARD,      SequenceInput.ARM_DONE,                   "prepareToIntake",           SequenceState.INTAKING},
        {SequenceState.INTAKING,                SequenceInput.DETECTED_PIECE,             "moveElevatorHome",          SequenceState.LOWERING},
        {SequenceState.LOWERING,                SequenceInput.ELEVATOR_THRESHOLD_MET,     "moveArmHome",               SequenceState.FINISHING},
        {SequenceState.FINISHING,               SequenceInput.RESET_DONE,                 "resetState",                SequenceState.HOME},

        // Level change sequences
        {SequenceState.RAISING_ELEVATOR,        SequenceInput.LEVEL_CHANGED,              "updateElevator",            SequenceState.UPDATING_LEVEL},
        {SequenceState.MOVING_ARM_FORWARD,      SequenceInput.LEVEL_CHANGED,              "returnArmForUpdate",        SequenceState.MOVING_ARM_BACK},
        {SequenceState.INTAKING,                SequenceInput.LEVEL_CHANGED,              "returnArmForUpdate",        SequenceState.MOVING_ARM_BACK},
        {SequenceState.MOVING_ARM_BACK,         SequenceInput.ARM_DONE,                   "updateElevator",            SequenceState.UPDATING_LEVEL},
        {SequenceState.UPDATING_LEVEL,          SequenceInput.ELEVATOR_DONE,              "moveArmForward",            SequenceState.MOVING_ARM_FORWARD},

        // Abort sequences
        {SequenceState.RAISING_ELEVATOR,        SequenceInput.BUTTON_RELEASED,            "startReset",                SequenceState.FINISHING},
        {SequenceState.MOVING_ARM_FORWARD,      SequenceInput.BUTTON_RELEASED,            "startReset",                SequenceState.FINISHING},
        {SequenceState.INTAKING,                SequenceInput.BUTTON_RELEASED,            "stopIntaking",              SequenceState.STOPPING_INTAKE},
        {SequenceState.STOPPING_INTAKE,         SequenceInput.STOPPED_INTAKE,             "startReset",                SequenceState.FINISHING}
    };
     */
    

    public static Object [][] getTransitionTable() {
        return transitionTable;
    }
}
