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
            public final static double raiseElevatorPosition = 21*12/20;
            public final static double raiseElevatorThresholdPosition = 16*12/20;
            public final static double lowerElevatorThresholdPosition = 20*12/20;
            public final static double armForwardPosition = 8.5;
            public final static double armScorePosition = 15;
        }

        public static final class CORAL_L3 {
            public final static double raiseElevatorPosition = 46*12/20;
            public final static double raiseElevatorThresholdPosition = 41*12/20;
            public final static double lowerElevatorThresholdPosition = 45*12/20;
            public final static double armForwardPosition = 8.5;
            public final static double armScorePosition = 15;
        }

        public static final class CORAL_L4 {
            public final static double raiseElevatorPosition = 95*12/20;
            public final static double raiseElevatorThresholdPosition = 90*12/20;
            public final static double lowerElevatorThresholdPosition = 94*12/20;
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
            public final static double raiseElevatorPosition = 7*12/20;
            public final static double secondStageElevatorPosition = 17*12/20;
            public final static double clamperIntakePosition = reefIntakeWidth;
            public final static double clamperHoldPosition = algaeIntakeWidth;
        }

        public static final class ALGAE_REEF_L3 {
            public final static double raiseElevatorPosition = 33*12/20;
            public final static double secondStageElevatorPosition = 43*12/20;
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
            public final static double raiseElevatorPosition = 98*12/20;
            public final static double raiseElevatorThresholdPosition = 58*12/20;
            public final static double lowerElevatorThresholdPosition = 97*12/20;
            public final static double armForwardPosition = 5;
        }

        public static final class ALGAE_HANDOFF {
            public final static double armForwardPosition = 35;
        }
    }
}
