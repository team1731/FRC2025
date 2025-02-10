package frc.robot.state.sequencer;

import frc.robot.state.State;

public enum SequenceState implements State {
    HOME,
    RAISING_ELEVATOR,
    MOVING_ARM_FORWARD,
    WAITING, // for driver feedback or intake sensor feedback
    SCORING,
    INTAKING,
    STOPPING_INTAKE,
    LOWERING,
    FINISHING
}
