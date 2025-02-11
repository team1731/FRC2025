package frc.robot.state.sequencer;

import frc.robot.state.Input;

public enum SequenceInput implements Input {
    BEGIN,
    SCORE,
    RESET_DONE,
    BUTTON_RELEASED,
    LEVEL_CHANGED,

    // Subsystem feedback
    ELEVATOR_THRESHOLD_MET,
    ELEVATOR_DONE,
    ARM_DONE,
    HAND_DONE,
    DETECTED_PIECE,
    RELEASED_PIECE,
    STOPPED_INTAKE
}
