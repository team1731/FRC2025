package frc.robot.state.sequencer.positions;

import frc.robot.subsystems.hand.HandConstants;

public final class PositionConstants {
    public final static double clamperClosedPosition = HandConstants.clamperHomePosition;
    public final static double coralIntakeWidth = HandConstants.clamperCoralPosition;
    public final static double algaeIntakeWidth = HandConstants.clamperAlgaePosition;

    /*
     * !!!!!!!!!!!!!!!!!!!!!!!!
     * CORAL INTAKE POSITION CONSTANTS
     * !!!!!!!!!!!!!!!!!!!!!!!!
     */

    public static final class CORAL_INTAKE {
        private static final class CORAL_FEEDER {
            public final static double handClamperPosition = coralIntakeWidth;
        }

        private static final class CORAL_FLOOR_UPRIGHT {
            public final static double armForwardPosition = 35;
            public final static double handClamperPosition = coralIntakeWidth;
        }

        // Accessors
        public static final Positions FEEDER_PICKUP_ACCESSOR = PositionsFactory.setCoralFeederPickupPositions(CORAL_FEEDER.handClamperPosition);
        public static final Positions FLOOR_UPRIGHT_PICKUP_ACCESSOR = PositionsFactory.setCoralUprightFloorPickupPositions(
            CORAL_FLOOR_UPRIGHT.armForwardPosition, CORAL_FLOOR_UPRIGHT.handClamperPosition
        );
    }

    
    /*
     * !!!!!!!!!!!!!!!!!!!!!!!!
     * CORAL SCORE POSITION CONSTANTS
     * !!!!!!!!!!!!!!!!!!!!!!!!
     */

    public static final class CORAL_SCORE {
        // TODO needs real positions, we haven't worked on this yets
        private static final class CORAL_L1 {
            public final static double raiseElevatorPosition = 0;
            public final static double raiseElevatorThresholdPosition = 0;
            public final static double lowerElevatorThresholdPosition = 0;
            public final static double armForwardPosition = 0;
            public final static double armScorePosition = 0;
        }

        private static final class CORAL_L2 {
            public final static double raiseElevatorPosition = 24;
            public final static double raiseElevatorThresholdPosition = 19;
            public final static double lowerElevatorThresholdPosition = 23;
            public final static double armForwardPosition = 9;
            public final static double armScorePosition = 15;
        }

        private static final class CORAL_L3 {
            public final static double raiseElevatorPosition = 49;
            public final static double raiseElevatorThresholdPosition = 44;
            public final static double lowerElevatorThresholdPosition = 48;
            public final static double armForwardPosition = 9;
            public final static double armScorePosition = 15;
        }

        private static final class CORAL_L4 {
            public final static double raiseElevatorPosition = 97;
            public final static double raiseElevatorThresholdPosition = 92;
            public final static double lowerElevatorThresholdPosition = 96;
            public final static double armForwardPosition = 9;
            public final static double armScorePosition = 15;
        }

        // Accessors
        public static final Positions L1_ACCESSOR = PositionsFactory.setCoralScorePositions(
            CORAL_L1.raiseElevatorPosition, CORAL_L1.raiseElevatorThresholdPosition, CORAL_L1.lowerElevatorThresholdPosition, 
            CORAL_L1.armForwardPosition, CORAL_L1.armScorePosition
        );
        public static final Positions L2_ACCESSOR = PositionsFactory.setCoralScorePositions(
            CORAL_L2.raiseElevatorPosition, CORAL_L2.raiseElevatorThresholdPosition, CORAL_L2.lowerElevatorThresholdPosition, 
            CORAL_L2.armForwardPosition, CORAL_L2.armScorePosition
        );
        public static final Positions L3_ACCESSOR = PositionsFactory.setCoralScorePositions(
            CORAL_L3.raiseElevatorPosition, CORAL_L3.raiseElevatorThresholdPosition, CORAL_L3.lowerElevatorThresholdPosition, 
            CORAL_L3.armForwardPosition, CORAL_L3.armScorePosition
        );
        public static final Positions L4_ACCESSOR = PositionsFactory.setCoralScorePositions(
            CORAL_L4.raiseElevatorPosition, CORAL_L4.raiseElevatorThresholdPosition, CORAL_L4.lowerElevatorThresholdPosition, 
            CORAL_L4.armForwardPosition, CORAL_L4.armScorePosition
        );
    }


    /*
     * !!!!!!!!!!!!!!!!!!!!!!!!
     * ALGAE INTAKE POSITION CONSTANTS
     * !!!!!!!!!!!!!!!!!!!!!!!!
     */

    public static final class ALGAE_INTAKE {
        private static final class ALGAE_REEF_L2 {
            // position definitions
            public final static double raiseElevatorPosition = 44;
            public final static double raiseElevatorThresholdPosition = 39;
            public final static double lowerElevatorThresholdPosition = 43;
            public final static double armForwardPosition = 35;
            public final static double handClamperPosition = algaeIntakeWidth;
        }

        private static final class ALGAE_REEF_L3 {
            public final static double raiseElevatorPosition = 69;
            public final static double raiseElevatorThresholdPosition = 64;
            public final static double lowerElevatorThresholdPosition = 68;
            public final static double armForwardPosition = 35;
            public final static double handClamperPosition = algaeIntakeWidth;
        }

        private static final class ALGAE_FLOOR {
            public final static double armForwardPosition = 35;
            public final static double handClamperPosition = algaeIntakeWidth;
        }

        // Accessors
        public static final Positions REEF_PICKUP_L2 = PositionsFactory.setCoralScorePositions(
            ALGAE_REEF_L2.raiseElevatorPosition, ALGAE_REEF_L2.raiseElevatorThresholdPosition, ALGAE_REEF_L2.lowerElevatorThresholdPosition, 
            ALGAE_REEF_L2.armForwardPosition, ALGAE_REEF_L2.handClamperPosition
        );
        public static final Positions REEF_PICKUP_L3 = PositionsFactory.setCoralScorePositions(
            ALGAE_REEF_L3.raiseElevatorPosition, ALGAE_REEF_L3.raiseElevatorThresholdPosition, ALGAE_REEF_L3.lowerElevatorThresholdPosition, 
            ALGAE_REEF_L3.armForwardPosition, ALGAE_REEF_L3.handClamperPosition
        );
        public static final Positions FLOOR_PICKUP = PositionsFactory.setAlgaeFloorPickupPositions(
            ALGAE_FLOOR.armForwardPosition, ALGAE_FLOOR.handClamperPosition
        );
    }


    /*
     * !!!!!!!!!!!!!!!!!!!!!!!!
     * ALGAE SCORE POSITION CONSTANTS
     * !!!!!!!!!!!!!!!!!!!!!!!!
     */

    public static final class ALGAE_SCORE {
        private static final class ALGAE_SCORE_BARGE {
            public final static double raiseElevatorPosition = 98;
            public final static double raiseElevatorThresholdPosition = 58;
            public final static double lowerElevatorThresholdPosition = 97;
            public final static double armForwardPosition = 5;
        }

        private static final class ALGAE_HANDOFF {
            public final static double armForwardPosition = 35;
        }

        // Accessors
        public static final Positions SCORE_BARGE = PositionsFactory.setAlgaeScorePositions(
            ALGAE_SCORE_BARGE.raiseElevatorPosition, ALGAE_SCORE_BARGE.raiseElevatorThresholdPosition, ALGAE_SCORE_BARGE.lowerElevatorThresholdPosition, 
            ALGAE_SCORE_BARGE.armForwardPosition
        );
        public static final Positions HANDOFF = PositionsFactory.setAlgaeHandoffPositions(ALGAE_HANDOFF.armForwardPosition);
    }
}
