package frc.robot.state.score;

import frc.robot.state.Input;

public enum ScoreInput implements Input {
    BEGIN,
    STOP_INTAKE,
    CLOSE_HAND,
    IS_SAFE,

    // Subsystem feedback
    ELEVATOR_THRESHOLD_MET,
    ELEVATOR_DONE,
    ARM_DONE,
    HAND_DONE
}
