package frc.robot.state.sequencer.transitions;

import frc.robot.state.sequencer.SequenceInput;
import frc.robot.state.sequencer.SequenceState;

public class AlgaeHandoffTransitions {
    private static final Object transitionTable[][] = {
        // CURRENT                              INPUT                                     OPERATION                    NEXT
        {SequenceState.HOME,                    SequenceInput.BEGIN,                      "moveArmForward",            SequenceState.MOVING_ARM_FORWARD},
        {SequenceState.MOVING_ARM_FORWARD,      SequenceInput.ARM_DONE,                   null,                        SequenceState.WAITING},
        {SequenceState.WAITING,                 SequenceInput.BUTTON_RELEASED,            "handoffAlgae",              SequenceState.SCORING},
        {SequenceState.SCORING,                 SequenceInput.RELEASED_PIECE,             "moveArmHome",               SequenceState.FINISHING},
        {SequenceState.FINISHING,               SequenceInput.ARM_DONE,                   "resetState",                SequenceState.HOME},

        // Abort sequences
        {SequenceState.MOVING_ARM_FORWARD,      SequenceInput.BUTTON_RELEASED,            "moveArmHome",               SequenceState.FINISHING}
    };
    

    public static Object [][] getTransitionTable() {
        return transitionTable;
    }
}
