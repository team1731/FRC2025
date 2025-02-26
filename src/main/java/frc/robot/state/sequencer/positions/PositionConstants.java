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
            public final static double raiseElevatorPosition = 23;
            public final static double raiseElevatorThresholdPosition = 16;
            public final static double lowerElevatorThresholdPosition = 20;
            public final static double armForwardPosition = 11;
            public final static double armForwardThreshold = 6;
            public final static double armScorePosition = 15;
        }

        public static final class CORAL_L3 {
            public final static double raiseElevatorPosition = 54;
            public final static double raiseElevatorThresholdPosition = 47;
            public final static double lowerElevatorThresholdPosition = 49;
            public final static double armForwardPosition = 11;
            public final static double armForwardThreshold = 6;
            public final static double armScorePosition = 15;
        }

        public static final class CORAL_L4 {
            public final static double raiseElevatorPosition = 87;
            public final static double raiseElevatorThresholdPosition = 82;
            public final static double lowerElevatorThresholdPosition = 87;
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
            public final static double raiseElevatorPosition = 22;
            public final static double secondStageElevatorPosition = 31;
            public final static double clamperIntakePosition = reefIntakeWidth;
            public final static double clamperHoldPosition = algaeIntakeWidth;
        }

        public static final class ALGAE_REEF_L3 {
            public final static double raiseElevatorPosition = 48;
            public final static double secondStageElevatorPosition = 57;
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
