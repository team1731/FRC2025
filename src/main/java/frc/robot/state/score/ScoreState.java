package frc.robot.state.score;

import frc.robot.state.State;

public enum ScoreState implements State {
    HOME,
    RAISING_ELEVATOR,
    LOWERING_ELEVATOR,
    MOVING_ARM_FORWARD,
    MOVING_ARM_BACK,
    WAITING,
    SCORING,
    CHECKING_SAFETY
}
