package frc.robot.state.sequencer;

import frc.robot.state.State;

public enum SequenceState implements State {
    HOME,
    DRIVING_TO_TARGET,
    RAISING_ELEVATOR,
    MOVING_TO_2ND_STAGE,
    LOWERING,
    MOVING_ARM_FORWARD,
    MOVING_ARM_BACK,
    WAITING, // for driver feedback or intake sensor feedback
    SCORING,
    AUTO_SCORE_WHEN_READY,
    INTAKING,
    STOPPING_INTAKE,
    OPENING_HAND,
    CLOSING_HAND,
    UPDATING_LEVEL,
    INIT_RESET,
    RESETTING,
    FINISHING
}
