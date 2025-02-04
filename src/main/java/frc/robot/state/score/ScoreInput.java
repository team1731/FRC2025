package frc.robot.state.score;

import frc.robot.state.Input;

public enum ScoreInput implements Input {
    BEGIN,
    GRABBED_PIECE,
    SCORE,
    RESET_DONE,

    // Subsystem feedback
    ELEVATOR_THRESHOLD_MET,
    ELEVATOR_DONE,
    ARM_DONE,
    HAND_DONE,
    DETECTED_PIECE,
    RELEASED_PIECE
}
