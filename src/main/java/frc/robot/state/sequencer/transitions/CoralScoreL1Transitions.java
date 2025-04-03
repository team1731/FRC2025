package frc.robot.state.sequencer.transitions;

import frc.robot.state.sequencer.SequenceInput;
import frc.robot.state.sequencer.SequenceState;

public class CoralScoreL1Transitions {
    private static final Object transitionTable[][] = {
        // CURRENT                              INPUT                                     OPERATION                    NEXT
        {SequenceState.HOME,                    SequenceInput.BEGIN,                      "raiseElevatorAndArmForL1",  SequenceState.RAISING_ELEVATOR},
        {SequenceState.RAISING_ELEVATOR,        SequenceInput.ELEVATOR_THRESHOLD_MET,     "moveArmSlowly",             SequenceState.WAITING},
        {SequenceState.WAITING,                 SequenceInput.BUTTON_RELEASED,            "openHand",                  SequenceState.SCORING},
        {SequenceState.SCORING,                 SequenceInput.HAND_DONE,                  "startIntakeResetSlow",          SequenceState.FINISHING},
        {SequenceState.FINISHING,               SequenceInput.RESET_DONE,                 "resetState",                SequenceState.HOME},
        
        // Abort sequences
        {SequenceState.RAISING_ELEVATOR,        SequenceInput.BUTTON_RELEASED,            "startIntakeResetSlow",          SequenceState.FINISHING},
        {SequenceState.MOVING_ARM_FORWARD,      SequenceInput.BUTTON_RELEASED,            "startIntakeResetSlow",          SequenceState.FINISHING}
    };
    

    public static Object [][] getTransitionTable() {
        return transitionTable;
    }
}
