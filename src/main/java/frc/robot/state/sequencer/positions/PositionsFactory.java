package frc.robot.state.sequencer.positions;

import frc.robot.state.sequencer.positions.PositionConstants.ALGAE_INTAKE.ALGAE_FLOOR;
import frc.robot.state.sequencer.positions.PositionConstants.ALGAE_INTAKE.ALGAE_REEF_L2;
import frc.robot.state.sequencer.positions.PositionConstants.ALGAE_INTAKE.ALGAE_REEF_L3;
import frc.robot.state.sequencer.positions.PositionConstants.ALGAE_SCORE.ALGAE_HANDOFF;
import frc.robot.state.sequencer.positions.PositionConstants.ALGAE_SCORE.ALGAE_SCORE_BARGE;
import frc.robot.state.sequencer.positions.PositionConstants.CORAL_INTAKE.CORAL_FEEDER;
import frc.robot.state.sequencer.positions.PositionConstants.CORAL_INTAKE.CORAL_FLOOR_UPRIGHT;
import frc.robot.state.sequencer.positions.PositionConstants.CORAL_SCORE.CORAL_L1;
import frc.robot.state.sequencer.positions.PositionConstants.CORAL_SCORE.CORAL_L2;
import frc.robot.state.sequencer.positions.PositionConstants.CORAL_SCORE.CORAL_L3;
import frc.robot.state.sequencer.positions.PositionConstants.CORAL_SCORE.CORAL_L4;


public class PositionsFactory {
    /*
     * !!!!!!!!!!!!!!!!!!!!!!
     * CORAL POSITION HELPERS
     * !!!!!!!!!!!!!!!!!!!!!!
     */

     public static Positions getCoralFeederPickupPositions() {
        Positions positions = new Positions();
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
        positions.lowerElevatorThreshold = CORAL_L1.lowerElevatorThresholdPosition;
        positions.firstStageArmPosition = CORAL_L1.armForwardPosition;
        positions.secondStageArmPosition = CORAL_L1.armScorePosition;
        return positions;
    }

    public static Positions getCoralScoreL2Positions() {
        Positions positions = new Positions();
        positions.raiseElevatorPosition = CORAL_L2.raiseElevatorPosition;
        positions.raiseElevatorThreshold = CORAL_L2.raiseElevatorThresholdPosition;
        positions.lowerElevatorThreshold = CORAL_L2.lowerElevatorThresholdPosition;
        positions.firstStageArmPosition = CORAL_L2.armForwardPosition;
        positions.secondStageArmPosition = CORAL_L2.armScorePosition;
        return positions;
    }

    public static Positions getCoralScoreL3Positions() {
        Positions positions = new Positions();
        positions.raiseElevatorPosition = CORAL_L3.raiseElevatorPosition;
        positions.raiseElevatorThreshold = CORAL_L3.raiseElevatorThresholdPosition;
        positions.lowerElevatorThreshold = CORAL_L3.lowerElevatorThresholdPosition;
        positions.firstStageArmPosition = CORAL_L3.armForwardPosition;
        positions.secondStageArmPosition = CORAL_L3.armScorePosition;
        return positions;
    }

    public static Positions getCoralScoreL4Positions() {
        Positions positions = new Positions();
        positions.raiseElevatorPosition = CORAL_L4.raiseElevatorPosition;
        positions.raiseElevatorThreshold = CORAL_L4.raiseElevatorThresholdPosition;
        positions.lowerElevatorThreshold = CORAL_L4.lowerElevatorThresholdPosition;
        positions.firstStageArmPosition = CORAL_L4.armForwardPosition;
        positions.secondStageArmPosition = CORAL_L4.armScorePosition;
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
        return positions;
    }

    public static Positions getAlgaeReefL3PickupPositions() {
        Positions positions = new Positions();
        positions.raiseElevatorPosition = ALGAE_REEF_L3.raiseElevatorPosition;
        positions.secondStageElevatorPosition = ALGAE_REEF_L3.secondStageElevatorPosition;
        positions.clamperIntakePosition = ALGAE_REEF_L3.clamperIntakePosition;
        positions.clamperHoldPosition = ALGAE_REEF_L3.clamperHoldPosition;
        return positions;
    }

    public static Positions getAlgaeFloorPickupPositions() {
        Positions positions = new Positions();
        positions.firstStageArmPosition = ALGAE_FLOOR.armForwardPosition;
        positions.clamperIntakePosition = ALGAE_FLOOR.clamperIntakePosition;
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
