package frc.robot.state.sequencer.transitions;

import frc.robot.state.sequencer.SequenceInput;
import frc.robot.state.sequencer.SequenceState;

public class CoralFeederPickupTransitions {
    private static final Object transitionTable[][] = {
        // CURRENT                              INPUT                                     OPERATION                    NEXT
        {SequenceState.HOME,                    SequenceInput.BEGIN,                      "moveArm",                   SequenceState.MOVING_ARM_BACK},
        {SequenceState.MOVING_ARM_BACK,         SequenceInput.ARM_DONE,                   "prepareToIntake",           SequenceState.INTAKING},
        {SequenceState.INTAKING,                SequenceInput.BUTTON_RELEASED,            "coralTimedIntake",          SequenceState.WAITING},
        {SequenceState.WAITING,                 SequenceInput.DETECTED_PIECE,             "startIntakeReset",                SequenceState.FINISHING},
        {SequenceState.WAITING,                 SequenceInput.TIMER_DONE,                 "startIntakeReset",                SequenceState.FINISHING},
        {SequenceState.FINISHING,               SequenceInput.RESET_DONE,                 "resetState",                SequenceState.HOME},

        // Abort sequences
        {SequenceState.MOVING_ARM_BACK,         SequenceInput.BUTTON_RELEASED,            "startReset",                SequenceState.FINISHING}
    };
    

    public static Object [][] getTransitionTable() {
        return transitionTable;
    }
}