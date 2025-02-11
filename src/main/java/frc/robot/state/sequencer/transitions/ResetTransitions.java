package frc.robot.state.sequencer.transitions;

import frc.robot.state.sequencer.SequenceInput;
import frc.robot.state.sequencer.SequenceState;

public class ResetTransitions {
    private static final Object transitionTable[][] = {
        // CURRENT                              INPUT                                     OPERATION                    NEXT
        {SequenceState.INIT_RESET,              SequenceInput.BEGIN,                      "startIntakeReset",          SequenceState.RESETTING},
        {SequenceState.RESETTING,               SequenceInput.RESET_DONE,                 "resetState",                SequenceState.HOME},
    };
    

    public static Object [][] getTransitionTable() {
        return transitionTable;
    }
}
