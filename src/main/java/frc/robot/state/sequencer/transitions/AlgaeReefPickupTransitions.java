package frc.robot.state.sequencer.transitions;

import frc.robot.state.sequencer.SequenceInput;
import frc.robot.state.sequencer.SequenceState;

public class AlgaeReefPickupTransitions {
    private static final Object transitionTable[][] = {
        // CURRENT                              INPUT                                     OPERATION                    NEXT
        {SequenceState.HOME,                    SequenceInput.BEGIN,                      "raiseElevatorNoThreshold",  SequenceState.RAISING_ELEVATOR},
        {SequenceState.RAISING_ELEVATOR,        SequenceInput.ELEVATOR_DONE,              "prepareToIntake",           SequenceState.INTAKING},
        {SequenceState.INTAKING,                SequenceInput.BUTTON_RELEASED,            "elevatorSecondStage",       SequenceState.UPDATING_ELEVATOR},
        {SequenceState.INTAKING,                SequenceInput.FINISH_INTAKE,              "elevatorSecondStage",       SequenceState.UPDATING_ELEVATOR},
        {SequenceState.UPDATING_ELEVATOR,       SequenceInput.ELEVATOR_DONE,              "holdAndLower",              SequenceState.LOWERING},
        {SequenceState.LOWERING,                SequenceInput.ELEVATOR_DONE,              "resetState",                SequenceState.HOME},

        // Level change sequences
        {SequenceState.RAISING_ELEVATOR,        SequenceInput.LEVEL_CHANGED,              "updateElevator",            SequenceState.UPDATING_LEVEL},
        {SequenceState.INTAKING,                SequenceInput.LEVEL_CHANGED,              "closeHand",                 SequenceState.STOPPING_INTAKE},
        {SequenceState.STOPPING_INTAKE,         SequenceInput.HAND_DONE,                  "updateElevator",            SequenceState.UPDATING_LEVEL},
        {SequenceState.UPDATING_LEVEL,          SequenceInput.ELEVATOR_DONE,              "prepareToIntake",           SequenceState.INTAKING},

        // Abort sequences
        {SequenceState.RAISING_ELEVATOR,        SequenceInput.BUTTON_RELEASED,            "startReset",               SequenceState.LOWERING}
    };
    

    public static Object [][] getTransitionTable() {
        return transitionTable;
    }
}
