package frc.robot.state.sequencer.transitions;

import frc.robot.state.sequencer.SequenceInput;
import frc.robot.state.sequencer.SequenceState;

public class CoralScoreL1Transitions {
    private static final Object transitionTable[][] = {
        // CURRENT                              INPUT                                     OPERATION                    NEXT
        {SequenceState.HOME,                    SequenceInput.BEGIN,                      "raiseElevator",             SequenceState.RAISING_ELEVATOR},
        {SequenceState.RAISING_ELEVATOR,        SequenceInput.ELEVATOR_THRESHOLD_MET,     "openHand",                  SequenceState.OPENING_HAND},
        {SequenceState.OPENING_HAND,            SequenceInput.HAND_DONE,                  null,                        SequenceState.WAITING},
        {SequenceState.WAITING,                 SequenceInput.BUTTON_RELEASED,            "releasePiece",              SequenceState.SCORING},
        {SequenceState.WAITING,                 SequenceInput.SCORE,                      "releasePiece",              SequenceState.SCORING},
        {SequenceState.SCORING,                 SequenceInput.RELEASED_PIECE,             "closeHand",                 SequenceState.CLOSING_HAND},
        {SequenceState.CLOSING_HAND,            SequenceInput.HAND_DONE,                  "startReset",                SequenceState.FINISHING},
        {SequenceState.FINISHING,               SequenceInput.RESET_DONE,                 "resetState",                SequenceState.HOME},
        
        // Abort sequences
        {SequenceState.RAISING_ELEVATOR,        SequenceInput.BUTTON_RELEASED,            "startReset",                SequenceState.FINISHING},
        {SequenceState.OPENING_HAND,            SequenceInput.BUTTON_RELEASED,            "closeHand",                 SequenceState.CLOSING_HAND}
    };
    

    public static Object [][] getTransitionTable() {
        return transitionTable;
    }
}
