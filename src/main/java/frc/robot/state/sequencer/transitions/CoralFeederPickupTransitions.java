package frc.robot.state.sequencer.transitions;

import frc.robot.state.sequencer.SequencerInput;
import frc.robot.state.sequencer.SequencerState;

public class CoralFeederPickupTransitions {
    private static final Object transitionTable[][] = {
        // CURRENT                           INPUT                                  OPERATION                    NEXT
        {SequencerState.HOME,                    SequencerInput.BEGIN,                      "prepareToIntake",           SequencerState.INTAKING},
        {SequencerState.INTAKING,                SequencerInput.DETECTED_PIECE,             "doSafetyCheck",             SequencerState.HOME}
    };
    

    public static Object [][] getTransitionTable() {
        return transitionTable;
    }
}
