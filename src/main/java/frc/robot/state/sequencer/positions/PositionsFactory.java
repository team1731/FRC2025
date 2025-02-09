package frc.robot.state.sequencer.positions;

public class PositionsFactory {
    /*
     * !!!!!!!!!!!!!!!!!!!!!!
     * CORAL POSITION HELPERS
     * !!!!!!!!!!!!!!!!!!!!!!
     */

     public static Positions setCoralFeederPickupPositions(double clamperPosition) {
        Positions positions = new Positions();
        positions.handClamperPosition = clamperPosition;
        return positions;
    }

    public static Positions setCoralUprightFloorPickupPositions(double armPosition, double clamperPosition) {
        Positions positions = new Positions();
        positions.armForwardPosition = armPosition;
        positions.handClamperPosition = clamperPosition;
        return positions;
    }

    public static Positions setCoralScorePositions(double raisePosition, double raiseThreshold, double lowerThreshold, double armPosition, double scorePosition) {
        Positions positions = new Positions();
        positions.raiseElevatorPosition = raisePosition;
        positions.raiseElevatorThreshold = raiseThreshold;
        positions.lowerElevatorThreshold = lowerThreshold;
        positions.armForwardPosition = armPosition;
        positions.armScoringPosition = scorePosition;
        return positions;
    }


    /*
     * !!!!!!!!!!!!!!!!!!!!!!
     * ALGAE POSITION HELPERS
     * !!!!!!!!!!!!!!!!!!!!!!
     */

    public static Positions setAlgaeReefPickupPositions(double raisePosition, double raiseThreshold, double lowerThreshold, double armPosition, double clamperPosition) {
        Positions positions = new Positions();
        positions.raiseElevatorPosition = raisePosition;
        positions.raiseElevatorThreshold = raiseThreshold;
        positions.lowerElevatorThreshold = lowerThreshold;
        positions.armForwardPosition = armPosition;
        positions.handClamperPosition = clamperPosition;
        return positions;
    }

    public static Positions setAlgaeFloorPickupPositions(double armPosition, double clamperPosition) {
        Positions positions = new Positions();
        positions.armForwardPosition = armPosition;
        positions.handClamperPosition = clamperPosition;
        return positions;
    }

    public static Positions setAlgaeScorePositions(double raisePosition, double raiseThreshold, double lowerThreshold, double armPosition) {
        Positions positions = new Positions();
        positions.raiseElevatorPosition = raisePosition;
        positions.raiseElevatorThreshold = raiseThreshold;
        positions.lowerElevatorThreshold = lowerThreshold;
        positions.armForwardPosition = armPosition;
        return positions;
    }

    public static Positions setAlgaeHandoffPositions(double armPosition) {
        Positions positions = new Positions();
        positions.armForwardPosition = armPosition;
        return positions;
    }
}
