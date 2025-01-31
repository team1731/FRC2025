package frc.robot.state.score;

import frc.robot.state.Input;

public enum ScoreInput implements Input {
    BEGIN,
    PIECE_SCORED,
    READY,

    // Subsystem feedback
    ELEVATOR_THRESHOLD,
    ELEVATOR_DONE,
    ARM_DONE
}
