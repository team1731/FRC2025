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
        public static final class CORAL_FEEDER {
            public final static double handClamperPosition = coralIntakeWidth;
        }

        public static final class CORAL_FLOOR_UPRIGHT {
            public final static double armForwardPosition = 35;
            public final static double handClamperPosition = coralIntakeWidth;
        }
    }

    
    /*
     * !!!!!!!!!!!!!!!!!!!!!!!!
     * CORAL SCORE POSITION CONSTANTS
     * !!!!!!!!!!!!!!!!!!!!!!!!
     */

    public static final class CORAL_SCORE {
        // TODO needs real positions, we haven't worked on this yets
        public static final class CORAL_L1 {
            public final static double raiseElevatorPosition = 0;
            public final static double raiseElevatorThresholdPosition = 0;
            public final static double lowerElevatorThresholdPosition = 0;
            public final static double armForwardPosition = 0;
            public final static double armScorePosition = 0;
        }

        public static final class CORAL_L2 {
            public final static double raiseElevatorPosition = 24;
            public final static double raiseElevatorThresholdPosition = 19;
            public final static double lowerElevatorThresholdPosition = 23;
            public final static double armForwardPosition = 9;
            public final static double armScorePosition = 15;
        }

        public static final class CORAL_L3 {
            public final static double raiseElevatorPosition = 49;
            public final static double raiseElevatorThresholdPosition = 44;
            public final static double lowerElevatorThresholdPosition = 48;
            public final static double armForwardPosition = 9;
            public final static double armScorePosition = 15;
        }

        public static final class CORAL_L4 {
            public final static double raiseElevatorPosition = 97;
            public final static double raiseElevatorThresholdPosition = 92;
            public final static double lowerElevatorThresholdPosition = 96;
            public final static double armForwardPosition = 9;
            public final static double armScorePosition = 15;
        }
    }


    /*
     * !!!!!!!!!!!!!!!!!!!!!!!!
     * ALGAE INTAKE POSITION CONSTANTS
     * !!!!!!!!!!!!!!!!!!!!!!!!
     */

    public static final class ALGAE_INTAKE {
        public static final class ALGAE_REEF_L2 {
            // position definitions
            public final static double raiseElevatorPosition = 44;
            public final static double raiseElevatorThresholdPosition = 39;
            public final static double lowerElevatorThresholdPosition = 43;
            public final static double armForwardPosition = 35;
            public final static double handClamperPosition = algaeIntakeWidth;
        }

        public static final class ALGAE_REEF_L3 {
            public final static double raiseElevatorPosition = 69;
            public final static double raiseElevatorThresholdPosition = 64;
            public final static double lowerElevatorThresholdPosition = 68;
            public final static double armForwardPosition = 35;
            public final static double handClamperPosition = algaeIntakeWidth;
        }

        public static final class ALGAE_FLOOR {
            public final static double armForwardPosition = 35;
            public final static double handClamperPosition = algaeIntakeWidth;
        }
    }


    /*
     * !!!!!!!!!!!!!!!!!!!!!!!!
     * ALGAE SCORE POSITION CONSTANTS
     * !!!!!!!!!!!!!!!!!!!!!!!!
     */

    public static final class ALGAE_SCORE {
        public static final class ALGAE_SCORE_BARGE {
            public final static double raiseElevatorPosition = 98;
            public final static double raiseElevatorThresholdPosition = 58;
            public final static double lowerElevatorThresholdPosition = 97;
            public final static double armForwardPosition = 5;
        }

        public static final class ALGAE_HANDOFF {
            public final static double armForwardPosition = 35;
        }
    }
}
