package frc.robot.state.sequencer.positions;

import frc.robot.subsystems.hand.HandConstants;

public final class PositionConstants {
    public final static double clamperClosedPosition = HandConstants.clamperHomePosition;
    public final static double coralIntakeWidth = HandConstants.clamperCoralPosition;
    public final static double algaeIntakeWidth = HandConstants.clamperAlgaePosition;
    public final static double reefIntakeWidth = HandConstants.clamperReefIntakePosition;

    /*
     * !!!!!!!!!!!!!!!!!!!!!!!!
     * CORAL INTAKE POSITION CONSTANTS
     * !!!!!!!!!!!!!!!!!!!!!!!!
     */

    public static final class CORAL_INTAKE {
        public static final class CORAL_FEEDER {
            public final static double armBackPosition = -2;
            public final static double armForwardPosition = 2;
            public final static double clamperIntakePosition = coralIntakeWidth;
        }

        public static final class CORAL_FLOOR_UPRIGHT {
            public final static double armForwardPosition = 35;
            public final static double clamperIntakePosition = coralIntakeWidth;
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
            public final static double raiseElevatorPosition = 21;
            public final static double raiseElevatorThresholdPosition = 16;
            public final static double lowerElevatorThresholdPosition = 20;
            public final static double armForwardPosition = 8.5;
            public final static double armScorePosition = 11;
        }

        public static final class CORAL_L3 {
            public final static double raiseElevatorPosition = 46;
            public final static double raiseElevatorThresholdPosition = 41;
            public final static double lowerElevatorThresholdPosition = 45;
            public final static double armForwardPosition = 8.5;
            public final static double armScorePosition = 13;
        }

        public static final class CORAL_L4 {
            public final static double raiseElevatorPosition = 95;
            public final static double raiseElevatorThresholdPosition = 90;
            public final static double lowerElevatorThresholdPosition = 95;
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
            public final static double raiseElevatorPosition = 7;
            public final static double secondStageElevatorPosition = 17;
            public final static double clamperIntakePosition = reefIntakeWidth;
            public final static double clamperHoldPosition = algaeIntakeWidth;
        }

        public static final class ALGAE_REEF_L3 {
            public final static double raiseElevatorPosition = 33;
            public final static double secondStageElevatorPosition = 43;
            public final static double clamperIntakePosition = reefIntakeWidth;
            public final static double clamperHoldPosition = algaeIntakeWidth;
        }

        public static final class ALGAE_FLOOR {
            public final static double armForwardPosition = 35;
            public final static double clamperIntakePosition = algaeIntakeWidth;
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
