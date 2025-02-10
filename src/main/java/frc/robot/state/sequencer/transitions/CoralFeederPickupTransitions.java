package frc.robot.state.sequencer.transitions;

import frc.robot.state.sequencer.SequenceInput;
import frc.robot.state.sequencer.SequenceState;

public class CoralFeederPickupTransitions {
    private static final Object transitionTable[][] = {
        // CURRENT                           INPUT                                  OPERATION                    NEXT
        {SequenceState.HOME,                    SequenceInput.BEGIN,                      "prepareToIntake",           SequenceState.INTAKING},
        {SequenceState.INTAKING,                SequenceInput.DETECTED_PIECE,             "doSafetyCheck",             SequenceState.HOME}
    };
    

    public static Object [][] getTransitionTable() {
        return transitionTable;
    }
}
