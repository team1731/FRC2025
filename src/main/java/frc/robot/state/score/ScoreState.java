package frc.robot.state.score;

import frc.robot.state.State;

public enum ScoreState implements State {
    HOME,
    RAISING_ELEVATOR,
    MOVING_ARM_FORWARD,
    WAITING,
    SCORING,
    LOWERING,
    FINISHING,
    ABORTING
}
