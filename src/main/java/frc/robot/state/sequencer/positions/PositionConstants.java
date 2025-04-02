package frc.robot.state.sequencer.positions;

import frc.robot.subsystems.hand.HandConstants;

public final class PositionConstants {
    public final static double clamperClosedPosition = HandConstants.clamperHomePosition;
    public final static double coralIntakeWidth = HandConstants.clamperCoralPosition;
    public final static double algaeIntakeWidth = HandConstants.clamperAlgaePosition;
    public final static double reefIntakeWidth = HandConstants.clamperReefIntakePosition;
    public final static double dumpCoralWidth = HandConstants.clamperDumpCoralPosition;
    public final static double pluckAlgaeWidth = HandConstants.clamperPluckAlgaePosition;

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
            public final static double raiseElevatorPosition = 42;
            public final static double raiseElevatorThresholdPosition = 35;
            public final static double armForwardPosition = 26;
            public final static double armForwardThreshold = 21;
            public final static double clamperOpenPosition = coralIntakeWidth;
        }

        public static final class CORAL_L1_ALT {
            public final static double raiseElevatorPosition = 6;
            public final static double raiseElevatorThresholdPosition = 2;
            public final static double clamperOpenPosition = dumpCoralWidth;
        }

        public static final class CORAL_L2 {
            public final static double raiseElevatorPosition = 21;
            public final static double raiseElevatorThresholdPosition = 20;
            public final static double lowerElevatorThresholdPosition = 18;
            public final static double armForwardPosition = 10;
            public final static double armForwardThreshold = 9;
            public final static double armScorePosition = 11;
        }

        public static final class CORAL_L3 {
            public final static double raiseElevatorPosition = 51;
            public final static double raiseElevatorThresholdPosition = 44;
            public final static double lowerElevatorThresholdPosition = 45;
            public final static double armForwardPosition = 11;
            public final static double armForwardThreshold = 6;
            public final static double armScorePosition = 15;
        }

        public static final class CORAL_L4 {
            public final static double raiseElevatorPosition = 93.5;
            public final static double raiseElevatorThresholdPosition = 68;  
            public final static double lowerElevatorThresholdPosition = 85.0;
            public final static double armForwardPosition = 9;
            public final static double armForwardThreshold = 9;
            public final static double armScorePosition = 15;
            public final static double clamperPluckAlgaePosition = pluckAlgaeWidth;
            public final static double thirdStageArmPosition = -3;
            public final static double clamperJigglePosition = 0.07;
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
            public final static double raiseElevatorPosition = 12;
            public final static double secondStageElevatorPosition = 15;
            public final static double clamperIntakePosition = reefIntakeWidth;
            public final static double clamperHoldPosition = algaeIntakeWidth;
            public final static double secondStageArmPosition = -3;
            public final static double clamperJigglePosition = 0.07;
        }

        public static final class ALGAE_REEF_L3 {
            public final static double raiseElevatorPosition = 42;
            public final static double secondStageElevatorPosition = 45;
            public final static double clamperIntakePosition = reefIntakeWidth;
            public final static double clamperHoldPosition = algaeIntakeWidth;
            public final static double secondStageArmPosition = -3; 
            public final static double clamperJigglePosition = 0.07; 
        }

        public static final class ALGAE_FLOOR {
            public final static double armForwardPosition = 15;
            public final static double clamperIntakePosition = algaeIntakeWidth;
            public final static double secondStageArmPosition = -3; 
            public final static double clamperJigglePosition = 0.07; 
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
            public final static double raiseElevatorThresholdPosition = 70;
            public final static double lowerElevatorThresholdPosition = 97;
            public final static double armForwardPosition = 8;
        }

        public static final class ALGAE_HANDOFF {
            public final static double armForwardPosition = 17;
        }
    }

    public static final class RESET {
        // reset positions
        public static final class UNSTUCK {
            public final static double raiseElevatorPosition = 23;
        }
    }
}
