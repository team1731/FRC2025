package frc.robot.state.score;

import frc.robot.state.Input;

public enum ScoreInput implements Input {
    BEGIN,
    PIECE_SCORED,
    IS_SAFE,

    // Subsystem feedback
    ELEVATOR_THRESHOLD_MET,
    ELEVATOR_DONE,
    ARM_DONE
}
