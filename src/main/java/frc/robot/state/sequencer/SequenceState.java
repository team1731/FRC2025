package frc.robot.state.sequencer;

import frc.robot.state.State;

public enum SequenceState implements State {
    HOME,
    RAISING_ELEVATOR,
    MOVING_TO_2ND_STAGE,
    LOWERING,
    MOVING_ARM_FORWARD,
    MOVING_ARM_BACK,
    WAITING, // for driver feedback or intake sensor feedback
    SCORING,
    INTAKING,
    STOPPING_INTAKE,
    OPENING_HAND,
    CLOSING_HAND,
    UPDATING_LEVEL,
    INIT_RESET,
    RESETTING,
    FINISHING
}
