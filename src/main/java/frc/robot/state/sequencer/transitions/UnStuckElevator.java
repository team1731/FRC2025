package frc.robot.state.sequencer.transitions;

import frc.robot.state.sequencer.SequenceInput;
import frc.robot.state.sequencer.SequenceState;

public class UnStuckElevator {
    private static final Object transitionTable[][] = {
        // CURRENT                              INPUT                                     OPERATION                    NEXT
        {SequenceState.HOME,                    SequenceInput.BEGIN,                      "raiseElevator",             SequenceState.UNSTUCKING},
        {SequenceState.UNSTUCKING,              SequenceInput.BUTTON_RELEASED,            "moveElevatorHome",          SequenceState.FINISHING},
        {SequenceState.FINISHING,               SequenceInput.ELEVATOR_DONE,              "resetState",                SequenceState.HOME},
    };
    

    public static Object [][] getTransitionTable() {
        return transitionTable;
    }
}