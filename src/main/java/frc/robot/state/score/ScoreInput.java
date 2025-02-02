package frc.robot.state.score;

import frc.robot.state.Input;

public enum ScoreInput implements Input {
    BEGIN,
    SCORE,
    RESET_DONE,
    IS_SAFE,

    // Subsystem feedback
    ELEVATOR_THRESHOLD_MET,
    ELEVATOR_DONE,
    ARM_DONE,
    HAND_DONE
}
