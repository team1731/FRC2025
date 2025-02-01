package frc.robot.state.score;

import frc.robot.state.State;

public enum ScoreState implements State {
    HOME,
    HAND_OPENING,
    INTAKE_STARTED,
    INTAKE_STOPPED,
    HAND_CLOSING,
    RAISING_ELEVATOR,
    MOVING_ARM_FORWARD,
    ELEVATOR_LOWERING,
    ARM_MOVING_BACK,
    CHECKING_SAFETY
}
