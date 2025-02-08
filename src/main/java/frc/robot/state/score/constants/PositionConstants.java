package frc.robot.state.score.constants;

import frc.robot.subsystems.hand.HandConstants;

public final class PositionConstants {
    public final static double clamperClosedPosition = HandConstants.clamperHomePosition;
    public final static double coralIntakeWidth = HandConstants.clamperCoralPosition;
    public final static double algaeIntakeWidth = HandConstants.clamperAlgaePosition;

    /*
     * !!!!!!!!!!!!!!!!!!!!!!!!
     * CORAL POSITION CONSTANTS
     * !!!!!!!!!!!!!!!!!!!!!!!!
     */

    public final static ScorePositions CORAL_FEEDER_PICKUP_POSITIONS = ScorePositions.getCoralFeederPickupPositions(coralIntakeWidth);

    // TODO define once the transitions are ready, needs floor feeder intake to be implemented
    public final static ScorePositions CORAL_FLOOR_PICKUP_POSITIONS = null; 

    public final static ScorePositions CORAL_UPRIGHT_FLOOR_PICKUP_POSITIONS = ScorePositions.getCoralUprightFloorPickupPositions(
        35, 
        coralIntakeWidth
    );

    public final static ScorePositions CORAL_L1_SCORE_POSITIONS = ScorePositions.getCoralScorePositions(
        // TODO needs real positions
        24, 
        22, 
        19, 
        9,
        15
    );

    public final static ScorePositions CORAL_L2_SCORE_POSITIONS = ScorePositions.getCoralScorePositions(
        24, 
        19, 
        23, 
        9,
        15
    );

    public final static ScorePositions CORAL_L3_SCORE_POSITIONS = ScorePositions.getCoralScorePositions(
        49, 
        44, 
        48, 
        9,
        15
    );

    public final static ScorePositions CORAL_L4_SCORE_POSITIONS = ScorePositions.getCoralScorePositions(
        97, 
        92, 
        96, 
        9,
        15
    );


    /*
     * !!!!!!!!!!!!!!!!!!!!!!!!
     * ALGAE POSITION CONSTANTS
     * !!!!!!!!!!!!!!!!!!!!!!!!
     */

    public final static ScorePositions ALGAE_REEF_PICKUP_L2_POSITIONS = ScorePositions.getAlgaeReefPickupPositions(
        44, 
        39,  
        43, 
        35, 
        algaeIntakeWidth
    );

    public final static ScorePositions ALGAE_REEF_PICKUP_L3_POSITIONS = ScorePositions.getAlgaeReefPickupPositions(
        69, 
        64, 
        68, 
        35, 
        algaeIntakeWidth
    );

    public final static ScorePositions ALGAE_FLOOR_PICKUP_POSITIONS = ScorePositions.getAlgaeFloorPickupPositions(
        35, 
        algaeIntakeWidth
    );

    public final static ScorePositions ALGAE_SCORE_POSITIONS = ScorePositions.getAlgaeScorePositions(
        98, 
        58, 
        97, 
        5
    );

    public final static ScorePositions ALGAE_HANDOFF_POSITIONS = ScorePositions.getAlgaeHandoffPositions(
        35
    );
}
