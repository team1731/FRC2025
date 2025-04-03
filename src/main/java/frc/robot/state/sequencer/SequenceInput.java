package frc.robot.state.sequencer;

import frc.robot.state.Input;

public enum SequenceInput implements Input {
    BEGIN,
    FINISH_INTAKE,
    SENSOR_SCORE,
    AUTO_SCORE,
    RESET_DONE,
    BUTTON_RELEASED,
    LEVEL_CHANGED,
    PLUCK_ALGAE,
    READY_FOR_JIGGLE,

    // Subsystem feedback
    DRIVE_THRESHOLD_MET,
    DRIVE_DISABLED,
    DRIVE_DONE,
    ELEVATOR_THRESHOLD_MET,
    ELEVATOR_DONE,
    ARM_DONE,
    ARM_THRESHOLD_MET,
    HAND_DONE,
    DETECTED_PIECE,
    JIGGLE_DONE,
    TIMER_DONE,
    RELEASED_PIECE,
    STOPPED_INTAKE
}
