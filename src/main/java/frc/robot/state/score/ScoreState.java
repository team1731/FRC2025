package frc.robot.state.score;

import frc.robot.state.State;

public enum ScoreState implements State {
    HOME,
    RAISING_ELEVATOR,
    MOVING_ARM_FORWARD,
    WAITING, // for driver feedback or intake sensor feedback
    MOVING_TO_SCORE,
    SCORING,
    INTAKING,
    CLOSING_HAND,
    LOWERING,
    FINISHING, // typically moving arm back
    ABORTING // if driver ends command early
}
