package frc.robot.state.sequencer;

import frc.robot.state.Input;

public enum SequencerInput implements Input {
    BEGIN,
    GRABBED_PIECE,
    SCORE,
    SCORED,
    RESET_DONE,

    // Subsystem feedback
    ELEVATOR_THRESHOLD_MET,
    ELEVATOR_DONE,
    ARM_DONE,
    HAND_DONE,
    DETECTED_PIECE,
    RELEASED_PIECE
}
