package frc.robot.state.sequencer.transitions;

import frc.robot.state.sequencer.SequenceInput;
import frc.robot.state.sequencer.SequenceState;

public class CoralFeederPickupTransitions {
    private static final Object transitionTable[][] = {
        // CURRENT                              INPUT                                     OPERATION                    NEXT
        {SequenceState.HOME,                    SequenceInput.BEGIN,                      "prepareToIntake",           SequenceState.INTAKING},
        {SequenceState.INTAKING,                SequenceInput.DETECTED_PIECE,             "resetState",                SequenceState.HOME},

        // Abort sequences
        {SequenceState.INTAKING,                SequenceInput.BUTTON_RELEASED,            "stopIntaking",              SequenceState.STOPPING_INTAKE},
        {SequenceState.STOPPING_INTAKE,         SequenceInput.STOPPED_INTAKE,             "resetState",                SequenceState.HOME}
    };
    

    public static Object [][] getTransitionTable() {
        return transitionTable;
    }
}
