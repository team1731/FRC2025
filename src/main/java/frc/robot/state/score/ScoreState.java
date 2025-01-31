package frc.robot.state.score;

import frc.robot.state.State;

public enum ScoreState implements State {
    HOME,
    RAISING,
    POSITIONING,
    SCORING,
    ARM_RESETTING,
    LOWERING,
    CHECKING_SAFETY
}
