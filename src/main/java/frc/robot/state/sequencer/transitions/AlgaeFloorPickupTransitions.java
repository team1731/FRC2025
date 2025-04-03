package frc.robot.state.sequencer.transitions;

import frc.robot.state.sequencer.SequenceInput;
import frc.robot.state.sequencer.SequenceState;

public class AlgaeFloorPickupTransitions {
    private static final Object transitionTable[][] = {
        // CURRENT                              INPUT                                     OPERATION                    NEXT
        {SequenceState.HOME,                    SequenceInput.BEGIN,                      "moveArm",                   SequenceState.MOVING_ARM_FORWARD},
        {SequenceState.MOVING_ARM_FORWARD,      SequenceInput.ARM_DONE,                   "prepareToIntake",           SequenceState.INTAKING},
        {SequenceState.INTAKING,                SequenceInput.BUTTON_RELEASED,            "moveArmHomeSlowly",         SequenceState.LOWERING},

        {SequenceState.LOWERING,                SequenceInput.ARM_DONE,                   "armSecondStage",            SequenceState.MOVING_ARM_BACK}, 
        {SequenceState.MOVING_ARM_BACK,         SequenceInput.ARM_DONE,                   "algaeJiggle",               SequenceState.JIGGLING}, 
        {SequenceState.JIGGLING,                SequenceInput.TIMER_DONE,                 "algaeIntake",               SequenceState.HOME},


        // Abort sequences
        {SequenceState.MOVING_ARM_FORWARD,      SequenceInput.BUTTON_RELEASED,            "moveArmHome",               SequenceState.FINISHING}
    };
    

    public static Object [][] getTransitionTable() {
        return transitionTable;
    }
}