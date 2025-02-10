package frc.robot.state.sequencer.transitions;

import frc.robot.state.sequencer.SequenceInput;
import frc.robot.state.sequencer.SequenceState;

public class CoralScoreTransitions {
    private static final Object transitionTable[][] = {
        // CURRENT                              INPUT                                     OPERATION                    NEXT
        {SequenceState.HOME,                    SequenceInput.BEGIN,                      "raiseElevator",             SequenceState.RAISING_ELEVATOR},
        {SequenceState.RAISING_ELEVATOR,        SequenceInput.ELEVATOR_THRESHOLD_MET,     "moveArmForward",            SequenceState.MOVING_ARM_FORWARD},
        {SequenceState.MOVING_ARM_FORWARD,      SequenceInput.ARM_DONE,                   null,                        SequenceState.WAITING},
        {SequenceState.WAITING,                 SequenceInput.BUTTON_RELEASED,            "moveArmToScore",            SequenceState.SCORING},
        {SequenceState.SCORING,                 SequenceInput.ARM_DONE,                   "moveElevatorHome",          SequenceState.LOWERING},
        {SequenceState.LOWERING,                SequenceInput.ELEVATOR_THRESHOLD_MET,     "moveArmHome",               SequenceState.FINISHING},
        {SequenceState.FINISHING,               SequenceInput.RESET_DONE,                 "resetState",                SequenceState.HOME},
        
        // Abort sequences
        {SequenceState.RAISING_ELEVATOR,        SequenceInput.BUTTON_RELEASED,            "startReset",                SequenceState.FINISHING},
        {SequenceState.MOVING_ARM_FORWARD,      SequenceInput.BUTTON_RELEASED,            "startReset",                SequenceState.FINISHING}
    };
    

    public static Object [][] getTransitionTable() {
        return transitionTable;
    }
}
