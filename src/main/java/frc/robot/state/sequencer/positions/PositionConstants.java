package frc.robot.state.sequencer.positions;

import frc.robot.subsystems.hand.HandConstants;

public final class PositionConstants {
    public final static double clamperClosedPosition = HandConstants.clamperHomePosition;
    public final static double coralIntakeWidth = HandConstants.clamperCoralPosition;
    public final static double algaeIntakeWidth = HandConstants.clamperAlgaePosition;
    public final static double reefIntakeWidth = HandConstants.clamperReefIntakePosition;
    public final static double dumpCoralWidth = HandConstants.clamperDumpCoralPosition;

    /*
     * !!!!!!!!!!!!!!!!!!!!!!!!
     * CORAL INTAKE POSITION CONSTANTS
     * !!!!!!!!!!!!!!!!!!!!!!!!
     */

    public static final class CORAL_INTAKE {
        public static final class CORAL_FEEDER {
            public final static double armBackPosition = -.5;
            public final static double armForwardPosition = .5;
            public final static double clamperIntakePosition = coralIntakeWidth;
        }

        public static final class CORAL_FLOOR_UPRIGHT {
            public final static double armForwardPosition = 19; // TODO need to update all of these arm positions, what is the calc? it is 9/800
            public final static double clamperIntakePosition = coralIntakeWidth;
        }
    }

    
    /*
     * !!!!!!!!!!!!!!!!!!!!!!!!
     * CORAL SCORE POSITION CONSTANTS
     * !!!!!!!!!!!!!!!!!!!!!!!!
     */

    public static final class CORAL_SCORE {
        public static final class CORAL_L1 {
            public final static double raiseElevatorPosition = 21;
            public final static double raiseElevatorThresholdPosition = 10;
            public final static double armForwardPosition = 15;
            public final static double armForwardThreshold = 11;
        }

        public static final class CORAL_L1_ALT {
            public final static double raiseElevatorPosition = 6;
            public final static double raiseElevatorThresholdPosition = 2;
            public final static double clamperOpenPosition = dumpCoralWidth;
        }

        public static final class CORAL_L2 {
            public final static double raiseElevatorPosition = 21;
            public final static double raiseElevatorThresholdPosition = 16;
            public final static double lowerElevatorThresholdPosition = 20;
            public final static double armForwardPosition = 8.5;
            public final static double armForwardThreshold = 6;
            public final static double armScorePosition = 11;
        }

        public static final class CORAL_L3 {
            public final static double raiseElevatorPosition = 46;
            public final static double raiseElevatorThresholdPosition = 41;
            public final static double lowerElevatorThresholdPosition = 45;
            public final static double armForwardPosition = 8.5;
            public final static double armForwardThreshold = 6;
            public final static double armScorePosition = 13;
        }

        public static final class CORAL_L4 {
            public final static double raiseElevatorPosition = 95;
            public final static double raiseElevatorThresholdPosition = 90;
            public final static double lowerElevatorThresholdPosition = 95;
            public final static double armForwardPosition = 9;
            public final static double armForwardThreshold = 6;
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
            public final static double raiseElevatorPosition = 16;
            public final static double secondStageElevatorPosition = 23;
            public final static double clamperIntakePosition = reefIntakeWidth;
            public final static double clamperHoldPosition = algaeIntakeWidth;
        }

        public static final class ALGAE_REEF_L3 {
            public final static double raiseElevatorPosition = 42;
            public final static double secondStageElevatorPosition = 49;
            public final static double clamperIntakePosition = reefIntakeWidth;
            public final static double clamperHoldPosition = algaeIntakeWidth;
        }

        public static final class ALGAE_FLOOR {
            public final static double armForwardPosition = 19;
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
            public final static double armForwardPosition = 15;
        }
    }
}
