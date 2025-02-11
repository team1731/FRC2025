package frc.robot.state.sequencer;

import frc.robot.state.State;

public enum SequenceState implements State {
    HOME,
    RAISING_ELEVATOR,
    UPDATING_ELEVATOR,
    MOVING_ARM_FORWARD,
    MOVING_ARM_BACK,
    WAITING, // for driver feedback or intake sensor feedback
    SCORING,
    INTAKING,
    STOPPING_INTAKE,
    LOWERING,
    UPDATING_LEVEL,
    INIT_RESET,
    RESETTING,
    FINISHING
}
