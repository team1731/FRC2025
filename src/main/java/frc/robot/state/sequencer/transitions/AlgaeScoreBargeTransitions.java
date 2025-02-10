package frc.robot.state.sequencer.transitions;

import frc.robot.state.sequencer.SequencerInput;
import frc.robot.state.sequencer.SequencerState;

public class AlgaeScoreBargeTransitions {
    private static final Object transitionTable[][] = {
        // CURRENT                           INPUT                                  OPERATION                    NEXT
        {SequencerState.HOME,                    SequencerInput.BEGIN,                      "raiseElevator",             SequencerState.RAISING_ELEVATOR},
        {SequencerState.RAISING_ELEVATOR,        SequencerInput.ELEVATOR_THRESHOLD_MET,     "moveArmForward",            SequencerState.MOVING_ARM_FORWARD},
        {SequencerState.MOVING_ARM_FORWARD,      SequencerInput.ARM_DONE,                   "shootToScore",              SequencerState.SCORING},
        {SequencerState.SCORING,                 SequencerInput.RELEASED_PIECE,             "moveElevatorHome",          SequencerState.LOWERING},
        {SequencerState.LOWERING,                SequencerInput.ELEVATOR_THRESHOLD_MET,     "moveArmHome",               SequencerState.FINISHING},
        {SequencerState.FINISHING,               SequencerInput.RESET_DONE,                 "doSafetyCheck",             SequencerState.HOME},
        {SequencerState.ABORTING,                SequencerInput.RESET_DONE,                 "doSafetyCheck",             SequencerState.HOME}
    };
    

    public static Object [][] getTransitionTable() {
        return transitionTable;
    }
}
