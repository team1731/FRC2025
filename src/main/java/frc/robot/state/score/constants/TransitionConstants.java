package frc.robot.state.score.constants;

import frc.robot.state.score.ScoreInput;
import frc.robot.state.score.ScoreState;

public final class TransitionConstants {

    /*
     * !!!!!!!!!!!!!!!!!
     * CORAL TRANSITIONS
     * !!!!!!!!!!!!!!!!!
     */

    public static final Object PICKUP_CORAL_FROM_FEEDER_TRANSITION_TABLE[][] = {
        // CURRENT                           INPUT                                  OPERATION                    NEXT
        {ScoreState.HOME,                    ScoreInput.BEGIN,                      "prepareToIntake",           ScoreState.INTAKING},
        {ScoreState.INTAKING,                ScoreInput.DETECTED_PIECE,             "doSafetyCheck",             ScoreState.HOME}
    };

    // NOTE: REQUIRES FLOOR FEEDER TO BE IMPLEMENTED FIRST
    public static final Object PICKUP_CORAL_FROM_FLOOR_TRANSITION_TABLE[][] = {
        // CURRENT                           INPUT                                  OPERATION                    NEXT
        // TODO needs to be defined
    };

    
    public static final Object PICKUP_UPRIGHT_CORAL_FROM_FLOOR_TRANSITION_TABLE[][] = {
        // CURRENT                           INPUT                                  OPERATION                    NEXT
        {ScoreState.HOME,                    ScoreInput.BEGIN,                      "moveArmForward",             ScoreState.MOVING_ARM_FORWARD},
        {ScoreState.MOVING_ARM_FORWARD,      ScoreInput.ARM_DONE,                   "prepareToIntake",            ScoreState.INTAKING},
        {ScoreState.INTAKING,                ScoreInput.DETECTED_PIECE,             "moveArmHome",                ScoreState.FINISHING},
        {ScoreState.FINISHING,               ScoreInput.ARM_DONE,                   "doSafetyCheck",              ScoreState.HOME},
        {ScoreState.ABORTING,                ScoreInput.ARM_DONE,                   "doSafetyCheck",              ScoreState.HOME}    
    };

    public static final Object SCORE_CORAL_TRANSITION_TABLE[][] = {
        // CURRENT                           INPUT                                  OPERATION                    NEXT
        {ScoreState.HOME,                    ScoreInput.BEGIN,                      "raiseElevator",             ScoreState.RAISING_ELEVATOR},
        {ScoreState.RAISING_ELEVATOR,        ScoreInput.ELEVATOR_THRESHOLD_MET,     "moveArmForward",            ScoreState.MOVING_ARM_FORWARD},
        {ScoreState.MOVING_ARM_FORWARD,      ScoreInput.ARM_DONE,                   null,                        ScoreState.WAITING},
        {ScoreState.WAITING,                 ScoreInput.SCORE,                      "moveArmToScore",            ScoreState.SCORING},
        {ScoreState.SCORING,                 ScoreInput.ARM_DONE,                   "moveElevatorHome",          ScoreState.LOWERING},
        {ScoreState.LOWERING,                ScoreInput.ELEVATOR_THRESHOLD_MET,     "moveArmHome",               ScoreState.FINISHING},
        {ScoreState.FINISHING,               ScoreInput.RESET_DONE,                 "doSafetyCheck",             ScoreState.HOME},
        {ScoreState.ABORTING,                ScoreInput.RESET_DONE,                 "doSafetyCheck",             ScoreState.HOME}
    };




    /*
     * !!!!!!!!!!!!!!!!!
     * ALGAE TRANSITIONS
     * !!!!!!!!!!!!!!!!!
     */

    public static final Object PICKUP_ALGAE_FROM_REEF_TRANSITION_TABLE[][] = {
        // CURRENT                           INPUT                                  OPERATION                    NEXT
        {ScoreState.HOME,                    ScoreInput.BEGIN,                      "raiseElevator",             ScoreState.RAISING_ELEVATOR},
        {ScoreState.RAISING_ELEVATOR,        ScoreInput.ELEVATOR_THRESHOLD_MET,     "moveArmForward",            ScoreState.MOVING_ARM_FORWARD},
        {ScoreState.MOVING_ARM_FORWARD,      ScoreInput.ARM_DONE,                   "prepareToIntake",           ScoreState.INTAKING},
        {ScoreState.INTAKING,                ScoreInput.DETECTED_PIECE,             "moveElevatorHome",          ScoreState.LOWERING},
        {ScoreState.LOWERING,                ScoreInput.ELEVATOR_THRESHOLD_MET,     "moveArmHome",               ScoreState.FINISHING},
        {ScoreState.FINISHING,               ScoreInput.RESET_DONE,                 "doSafetyCheck",             ScoreState.HOME},
        {ScoreState.ABORTING,                ScoreInput.RESET_DONE,                 "doSafetyCheck",             ScoreState.HOME}
    };

    public static final Object PICKUP_ALGAE_FROM_FLOOR_TRANSITION_TABLE[][] = {
        // CURRENT                           INPUT                                  OPERATION                    NEXT
        {ScoreState.HOME,                    ScoreInput.BEGIN,                      "moveArmForward",            ScoreState.MOVING_ARM_FORWARD},
        {ScoreState.MOVING_ARM_FORWARD,      ScoreInput.ARM_DONE,                   "prepareToIntake",           ScoreState.INTAKING},
        {ScoreState.INTAKING,                ScoreInput.DETECTED_PIECE,             "moveArmHome",               ScoreState.FINISHING},
        {ScoreState.FINISHING,               ScoreInput.ARM_DONE,                   "doSafetyCheck",             ScoreState.HOME},
        {ScoreState.ABORTING,                ScoreInput.ARM_DONE,                   "doSafetyCheck",             ScoreState.HOME}
    };


    public static final Object SCORE_ALGAE_TRANSITION_TABLE[][] = {
        // CURRENT                           INPUT                                  OPERATION                    NEXT
        {ScoreState.HOME,                    ScoreInput.BEGIN,                      "raiseElevator",             ScoreState.RAISING_ELEVATOR},
        {ScoreState.RAISING_ELEVATOR,        ScoreInput.ELEVATOR_THRESHOLD_MET,     "moveArmForward",            ScoreState.MOVING_ARM_FORWARD},
        {ScoreState.MOVING_ARM_FORWARD,      ScoreInput.ARM_DONE,                   "shootToScore",              ScoreState.SCORING},
        {ScoreState.SCORING,                 ScoreInput.RELEASED_PIECE,             "moveElevatorHome",          ScoreState.LOWERING},
        {ScoreState.LOWERING,                ScoreInput.ELEVATOR_THRESHOLD_MET,     "moveArmHome",               ScoreState.FINISHING},
        {ScoreState.FINISHING,               ScoreInput.RESET_DONE,                 "doSafetyCheck",             ScoreState.HOME},
        {ScoreState.ABORTING,                ScoreInput.RESET_DONE,                 "doSafetyCheck",             ScoreState.HOME}
    };

    
    public static final Object HANDOFF_ALGAE_TRANSITION_TABLE[][] = {
        // CURRENT                           INPUT                                  OPERATION                    NEXT
        {ScoreState.HOME,                    ScoreInput.BEGIN,                      "moveArmForward",            ScoreState.MOVING_ARM_FORWARD},
        {ScoreState.MOVING_ARM_FORWARD,      ScoreInput.ARM_DONE,                   "shootToScore",              ScoreState.SCORING},
        {ScoreState.SCORING,                 ScoreInput.RELEASED_PIECE,             "moveArmHome",               ScoreState.FINISHING},
        {ScoreState.FINISHING,               ScoreInput.ARM_DONE,                   "doSafetyCheck",             ScoreState.HOME},
        {ScoreState.ABORTING,                ScoreInput.ARM_DONE,                   "doSafetyCheck",             ScoreState.HOME}
    };
}
