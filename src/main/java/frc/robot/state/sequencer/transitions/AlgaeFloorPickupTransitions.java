package frc.robot.state.sequencer.transitions;

import frc.robot.state.sequencer.SequenceInput;
import frc.robot.state.sequencer.SequenceState;

public class AlgaeFloorPickupTransitions {
    private static final Object transitionTable[][] = {
        // CURRENT                           INPUT                                  OPERATION                    NEXT
        {SequenceState.HOME,                    SequenceInput.BEGIN,                      "moveArmForward",            SequenceState.MOVING_ARM_FORWARD},
        {SequenceState.MOVING_ARM_FORWARD,      SequenceInput.ARM_DONE,                   "prepareToIntake",           SequenceState.INTAKING},
        {SequenceState.INTAKING,                SequenceInput.DETECTED_PIECE,             "moveArmHome",               SequenceState.FINISHING},
        {SequenceState.FINISHING,               SequenceInput.ARM_DONE,                   "doSafetyCheck",             SequenceState.HOME},
        {SequenceState.ABORTING,                SequenceInput.ARM_DONE,                   "doSafetyCheck",             SequenceState.HOME}
    };
    

    public static Object [][] getTransitionTable() {
        return transitionTable;
    }
}
