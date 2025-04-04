package frc.robot.state.sequencer.positions;

import frc.robot.state.sequencer.positions.PositionConstants.ALGAE_INTAKE.ALGAE_FLOOR;
import frc.robot.state.sequencer.positions.PositionConstants.ALGAE_INTAKE.ALGAE_REEF_L2;
import frc.robot.state.sequencer.positions.PositionConstants.ALGAE_INTAKE.ALGAE_REEF_L3;
import frc.robot.state.sequencer.positions.PositionConstants.ALGAE_SCORE.ALGAE_HANDOFF;
import frc.robot.state.sequencer.positions.PositionConstants.ALGAE_SCORE.ALGAE_SCORE_BARGE;
import frc.robot.state.sequencer.positions.PositionConstants.CORAL_INTAKE.CORAL_FEEDER;
import frc.robot.state.sequencer.positions.PositionConstants.CORAL_INTAKE.CORAL_FLOOR_UPRIGHT;
import frc.robot.state.sequencer.positions.PositionConstants.CORAL_SCORE.CORAL_L1;
import frc.robot.state.sequencer.positions.PositionConstants.CORAL_SCORE.CORAL_L1_ALT;
import frc.robot.state.sequencer.positions.PositionConstants.CORAL_SCORE.CORAL_L2;
import frc.robot.state.sequencer.positions.PositionConstants.CORAL_SCORE.CORAL_L3;
import frc.robot.state.sequencer.positions.PositionConstants.CORAL_SCORE.CORAL_L4;
import frc.robot.state.sequencer.positions.PositionConstants.RESET.UNSTUCK;


public class PositionsFactory {
    /*
     * !!!!!!!!!!!!!!!!!!!!!!
     * CORAL POSITION HELPERS
     * !!!!!!!!!!!!!!!!!!!!!!
     */

     public static Positions getCoralFeederPickupPositions() {
        Positions positions = new Positions();
        positions.firstStageArmPosition = CORAL_FEEDER.armBackPosition;
        positions.secondStageArmPosition = CORAL_FEEDER.armForwardPosition;
        positions.clamperIntakePosition = CORAL_FEEDER.clamperIntakePosition;
        return positions;
    }

    public static Positions getCoralUprightFloorPickupPositions() {
        Positions positions = new Positions();
        positions.firstStageArmPosition = CORAL_FLOOR_UPRIGHT.armForwardPosition;
        positions.clamperIntakePosition = CORAL_FLOOR_UPRIGHT.clamperIntakePosition;
        return positions;
    }

    public static Positions getCoralScoreL1Positions() {
        Positions positions = new Positions();
        positions.raiseElevatorPosition = CORAL_L1.raiseElevatorPosition;
        positions.raiseElevatorThreshold = CORAL_L1.raiseElevatorThresholdPosition;
        positions.firstStageArmPosition = CORAL_L1.armForwardPosition;
        positions.firstStageArmThreshold = CORAL_L1.armForwardThreshold;
        positions.clamperOpenPosition = CORAL_L1.clamperOpenPosition;
        return positions;
    }

    public static Positions getCoralScoreL1AltPositions() {
        Positions positions = new Positions();
        positions.raiseElevatorPosition = CORAL_L1_ALT.raiseElevatorPosition;
        positions.raiseElevatorThreshold = CORAL_L1_ALT.raiseElevatorThresholdPosition;
        positions.clamperOpenPosition = CORAL_L1_ALT.clamperOpenPosition;
        return positions;
    }

    public static Positions getCoralScoreL2Positions() {
        Positions positions = new Positions();
        positions.raiseElevatorPosition = CORAL_L2.raiseElevatorPosition;
        positions.raiseElevatorThreshold = CORAL_L2.raiseElevatorThresholdPosition;
        positions.lowerElevatorThreshold = CORAL_L2.lowerElevatorThresholdPosition;
        positions.firstStageArmPosition = CORAL_L2.armForwardPosition;
        positions.firstStageArmThreshold = CORAL_L2.armForwardThreshold;
        positions.secondStageArmPosition = CORAL_L2.armScorePosition;
        return positions;
    }

    public static Positions getCoralScoreL3Positions() {
        Positions positions = new Positions();
        positions.raiseElevatorPosition = CORAL_L3.raiseElevatorPosition;
        positions.raiseElevatorThreshold = CORAL_L3.raiseElevatorThresholdPosition;
        positions.lowerElevatorThreshold = CORAL_L3.lowerElevatorThresholdPosition;
        positions.firstStageArmPosition = CORAL_L3.armForwardPosition;
        positions.firstStageArmThreshold = CORAL_L3.armForwardThreshold;
        positions.secondStageArmPosition = CORAL_L3.armScorePosition;
        return positions;
    }

    public static Positions getCoralScoreL4Positions() {
        Positions positions = new Positions();
        positions.raiseElevatorPosition = CORAL_L4.raiseElevatorPosition;
        positions.raiseElevatorThreshold = CORAL_L4.raiseElevatorThresholdPosition;
        positions.lowerElevatorThreshold = CORAL_L4.lowerElevatorThresholdPosition;
        positions.firstStageArmPosition = CORAL_L4.armForwardPosition;
        positions.firstStageArmThreshold = CORAL_L4.armForwardThreshold;
        positions.secondStageArmPosition = CORAL_L4.armScorePosition;
        positions.secondStageArmThreshold = CORAL_L4.armScoreThreshold;
        positions.clamperOpenPosition = CORAL_L4.clamperPluckAlgaePosition;
        positions.thirdStageArmPosition = CORAL_L4.thirdStageArmPosition;
        positions.clamperJigglePosition = CORAL_L4.clamperJigglePosition;
        return positions;
    }

    public static Positions getUnStuckElevator() {
        Positions positions = new Positions();
        positions.raiseElevatorPosition = UNSTUCK.raiseElevatorPosition;
        return positions;
    }


    /*
     * !!!!!!!!!!!!!!!!!!!!!!
     * ALGAE POSITION HELPERS
     * !!!!!!!!!!!!!!!!!!!!!!
     */

    public static Positions getAlgaeReefL2PickupPositions() {
        Positions positions = new Positions();
        positions.raiseElevatorPosition = ALGAE_REEF_L2.raiseElevatorPosition;
        positions.secondStageElevatorPosition = ALGAE_REEF_L2.secondStageElevatorPosition;
        positions.clamperIntakePosition = ALGAE_REEF_L2.clamperIntakePosition;
        positions.clamperHoldPosition = ALGAE_REEF_L2.clamperHoldPosition;
        positions.clamperJigglePosition = ALGAE_REEF_L2.clamperJigglePosition;
        positions.secondStageArmPosition = ALGAE_REEF_L2.secondStageArmPosition;
        return positions;
    }

    public static Positions getAlgaeReefL3PickupPositions() {
        Positions positions = new Positions();
        positions.raiseElevatorPosition = ALGAE_REEF_L3.raiseElevatorPosition;
        positions.secondStageElevatorPosition = ALGAE_REEF_L3.secondStageElevatorPosition;
        positions.clamperIntakePosition = ALGAE_REEF_L3.clamperIntakePosition;
        positions.clamperHoldPosition = ALGAE_REEF_L3.clamperHoldPosition;
        positions.clamperJigglePosition = ALGAE_REEF_L3.clamperJigglePosition;
        positions.secondStageArmPosition = ALGAE_REEF_L3.secondStageArmPosition;
        return positions;
    }

    public static Positions getAlgaeFloorPickupPositions() {
        Positions positions = new Positions();
        positions.firstStageArmPosition = ALGAE_FLOOR.armForwardPosition;
        positions.clamperIntakePosition = ALGAE_FLOOR.clamperIntakePosition;
        positions.clamperJigglePosition = ALGAE_FLOOR.clamperJigglePosition;
        positions.secondStageArmPosition = ALGAE_FLOOR.secondStageArmPosition;
        return positions;
    }

    public static Positions getAlgaeScoreBargePositions() {
        Positions positions = new Positions();
        positions.raiseElevatorPosition = ALGAE_SCORE_BARGE.raiseElevatorPosition;
        positions.raiseElevatorThreshold = ALGAE_SCORE_BARGE.raiseElevatorThresholdPosition;
        positions.lowerElevatorThreshold = ALGAE_SCORE_BARGE.lowerElevatorThresholdPosition;
        positions.firstStageArmPosition = ALGAE_SCORE_BARGE.armForwardPosition;
        return positions;
    }

    public static Positions getAlgaeHandoffPositions() {
        Positions positions = new Positions();
        positions.firstStageArmPosition = ALGAE_HANDOFF.armForwardPosition;
        return positions;
    }
}
