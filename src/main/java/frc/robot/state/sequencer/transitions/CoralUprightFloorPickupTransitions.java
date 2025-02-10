package frc.robot.state.sequencer.transitions;

import frc.robot.state.sequencer.SequencerInput;
import frc.robot.state.sequencer.SequencerState;

public class CoralUprightFloorPickupTransitions {
    private static final Object transitionTable[][] = {
        // CURRENT                           INPUT                                  OPERATION                    NEXT
        {SequencerState.HOME,                    SequencerInput.BEGIN,                      "moveArmForward",             SequencerState.MOVING_ARM_FORWARD},
        {SequencerState.MOVING_ARM_FORWARD,      SequencerInput.ARM_DONE,                   "prepareToIntake",            SequencerState.INTAKING},
        {SequencerState.INTAKING,                SequencerInput.DETECTED_PIECE,             "moveArmHome",                SequencerState.FINISHING},
        {SequencerState.FINISHING,               SequencerInput.ARM_DONE,                   "doSafetyCheck",              SequencerState.HOME},
        {SequencerState.ABORTING,                SequencerInput.ARM_DONE,                   "doSafetyCheck",              SequencerState.HOME}    
    };
    

    public static Object [][] getTransitionTable() {
        return transitionTable;
    }
}
