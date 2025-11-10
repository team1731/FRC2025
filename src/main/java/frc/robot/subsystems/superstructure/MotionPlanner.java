package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.arm.*;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.hand.*;

public class MotionPlanner {
    private ArmSubsystem arm;
    private ElevatorSubsystem elevator;
    private HandClamperSubsystem hand;
    private HandIntakeSubsystem intake;

    public MotionPlanner(ArmSubsystem arm, ElevatorSubsystem elevator, HandClamperSubsystem hand, HandIntakeSubsystem intake) {
        this.arm = arm;
        this.elevator = elevator;
        this.hand = hand;
        this.intake = intake;
    }

    // =========================
    //       Safety Checks
    // =========================

    public boolean elevatorAboveThreshold(double threshold) {
        return elevator.getPosition() > threshold;
    }

    public boolean elevatorBelowThreshold(double threshold) {
        return elevator.getPosition() < threshold;
    }

    public boolean elevatorWithinRange(Positions pos) {
        return elevator.getPosition() > pos.raiseElevatorThreshold - 3.0 &&
               elevator.getPosition() < pos.raiseElevatorPosition + 3.0;
    }

    public boolean armPastThreshold(double threshold) {
        return arm.getArmPosition() > threshold;
    }

    public boolean armBelowThreshold(double threshold) {
        return arm.getArmPosition() < threshold;
    }

    public boolean armPastSmackReefThreshold() {
        return arm.getArmPosition() > ArmConstants.willSmackReefThreshold;
    }

    public boolean hasPiece() {
        return intake.hasPiece();
    }

    public boolean alignedToScore() {
        return intake.alignedToPole();
    }

    // =========================
    //     Movement Commands
    // =========================

    public Command moveElevator(double position, boolean slowed) {
        return elevator.moveCommand(position, slowed);
    }

    public Command homeElevator() {
        return elevator.homeCommand();
    }

    public Command moveArm(double position) {
        return arm.moveCommand(position, false);
    }

    public Command moveArmSlow(double position) {
        return arm.moveCommand(position, true);
    }

    public Command homeArm() {
        return moveArm(ArmConstants.armHomePosition);
    }

    public Command homeArmSlow() {
        return moveArmSlow(ArmConstants.armHomePosition);
    }

    public Command openHand(double position) {
        return hand.openCommand(position);
    }

    public Command closeClamp() {
        return hand.closeCommand();
    }

    public Command clearArmForClimb() {
        return moveArm(ArmConstants.halfedArmPosition);
    }

    public Command clearArm(Positions pos) {
        return Commands.either(
            moveArm(ArmConstants.armHomePosition),
            Commands.none(),
            () -> armPastSmackReefThreshold() && elevatorWithinRange(pos)
        );
    }

    public Command clearArmSlow(Positions pos) {
        return Commands.either(
            moveArmSlow(ArmConstants.armHomePosition),
            Commands.none(),
            () -> armPastSmackReefThreshold() && elevatorWithinRange(pos)
        );
    }

    public Command clearHand(Positions pos) {
        return Commands.either(
            hand.closeCommand(),
            Commands.none(),
            () -> hand.getPosition() > 0.02 && elevatorWithinRange(pos)
        );
    }

    public Command clearArmAndHand(Positions pos) {
        return Commands.parallel(
            clearArm(pos),
            clearHand(pos)
        );
    }

    public Command clearArmSlowAndHand(Positions pos) {
        return Commands.parallel(
            clearArm(pos),
            clearHand(pos)
        );
    }

    public Command homeAll() {
        return Commands.parallel(
            homeArm(),
            homeElevator(),
            closeClamp()
        );
    }

    public Command homeAllAlgaeHold() {
        return Commands.parallel(
            homeArm(),
            homeElevator(),
            clampAlgaeHold()
        );
    }

    public Command moveElevatorFirstStage(Positions pos) {
        return moveElevator(pos.raiseElevatorPosition, false);
    }

    public Command moveElevatorSecondStage(Positions pos) {
        return moveElevator(pos.secondStageElevatorPosition, false);
    }

    public Command moveArmFirstStage(Positions pos) {
        return moveArm(pos.firstStageArmPosition);
    }

    public Command moveArmFirstStageSlow(Positions pos) {
        return moveArmSlow(pos.firstStageArmPosition);
    }

    public Command moveArmSecondStage(Positions pos) {
        return moveArm(pos.secondStageArmPosition);
    }

    public Command moveArmSecondStageSlow(Positions pos) {
        return moveArmSlow(pos.secondStageArmPosition);
    }

    public Command openClamp(Positions pos) {
        return openHand(pos.clamperOpenPosition);
    }

    public Command openClampCoral() {
        return hand.holdCoralCommand();
    }

    public Command openClampReefAlgae(Positions pos) {
        return openHand(pos.clamperIntakePosition);
    }

    public Command clampHold(Positions pos) {
        return openHand(pos.clamperHoldPosition);
    }

    public Command clampAlgaeHold() {
        return hand.holdAlgaeCommand();
    }

    public Command raiseElevatorThenArmWithThreshold(Positions pos, boolean slow) {
        return Commands.parallel(
            moveElevatorFirstStage(pos),
            Commands.waitUntil(() -> elevatorAboveThreshold(pos.raiseElevatorThreshold))
            .andThen(Commands.either(
                moveArmFirstStageSlow(pos), 
                moveArmFirstStage(pos), 
                () -> slow
            ))
        );
    }

    public Command raiseElevatorThenOpenHandAlgae(Positions pos) {
        return Commands.sequence(
            moveElevatorFirstStage(pos),
            openClampReefAlgae(pos)
        );
    }

    public Command homeArmAfterElevatorBelowThreshold(Positions pos) {
        return homeElevator()
        .alongWith(
            Commands.waitUntil(() -> elevatorBelowThreshold(pos.lowerElevatorThreshold))
            .andThen(homeArm())
        );
    }

    // =========================
    //    Superstate Commands
    // =========================

    public Command intakeCoralFeeder() {
        return clearArmAndHand(PositionsFactory.getCoralFeederPickupPositions())
        .andThen(homeElevator().alongWith(
            intake.intakeCoralCommand(), 
            openClamp(PositionsFactory.getCoralFeederPickupPositions())
        ));
    }

    public Command intakeReefAlgae(Positions pos) {
        return clearArmSlowAndHand(pos)
        .andThen(raiseElevatorThenOpenHandAlgae(pos));
    }

    public Command intakeFloorAlgae() {
        return clearArmSlowAndHand(PositionsFactory.getAlgaeFloorPickupPositions())
        .andThen(homeElevator())
        .andThen(
            moveArmFirstStage(PositionsFactory.getAlgaeFloorPickupPositions())
            .alongWith(
                intake.intakeAlgaeCommand(),
                openClamp(PositionsFactory.getAlgaeFloorPickupPositions())
            )
        );
    }

    public Command initializeCoralScore(Positions pos, boolean isL1, boolean preventL4) {
        return raiseElevatorThenArmWithThreshold(pos, isL1);
    }

    // public boolean pluckAlgae() {
    //     handIntakeSubsystem.intakeWithCurrent();
    //     arm.moveArmSlowAlgae(0.3, 5.0, 1, subsystemCallback);
    //     return true;
    // }

    // public boolean moveArmForAlgaeJiggle() {
    //     arm.moveArmSlowSpeed(positions.thirdStageArmPosition, subsystemCallback);
    //     return true;
    // }

    // public boolean algaeJiggle(){
    //     System.out.println("Jiggling algae");
    //     handClamperSubsystem.moveHand(positions.clamperJigglePosition);
    //     handIntakeSubsystem.releaseWithVelocity(5, 0.2, subsystemCallback);
    //     return true;
    // }

    // public boolean algaeIntake(){
    //     System.out.println("Intaking algae");
    //     // handIntakeSubsystem.stop();
    //     handIntakeSubsystem.intakeWithCurrent(); 
    //     resetState();
    //     return true;
    // }

    // TODO - Add ability to pluck algae
    public Command finishCoralScoreHigh(Positions pos, boolean pluckAlgae) {
        return moveArmSecondStage(pos)
        .andThen(
            homeArmAfterElevatorBelowThreshold(pos)
        );
    }

    public Command finishCoralScoreL2() {
        return Commands.parallel(
            moveArmSecondStage(PositionsFactory.getCoralScoreL2Positions()),
            moveElevator(PositionsFactory.getCoralScoreL2Positions().lowerElevatorThreshold, false),
            openClampCoral(),
            intake.releaseCommand()
        );
    }

    public Command finishCoralScoreL1() {
        return openClampCoral()
        .alongWith(intake.releaseCommand());
    }

    public Command finishReefAlgaeIntake(Positions pos) {
        return moveElevatorSecondStage(pos)
        .andThen(
            clampHold(pos), 
            homeElevator(),
            moveArmSecondStage(pos)
            .alongWith(intake.holdCommand())
        );
    }

    public Command finishGroundAlgaeIntake() {
        return homeAll()
        .alongWith(intake.intakeAlgaeCommand())
        .andThen(
            moveArm(ArmConstants.minArmPosition)
            .alongWith(intake.holdCommand())
        );
    }

    //                 yield homeCommand()
    //                 .andThen(
    //                     // Step 1: Bring up elevator with arm back
    //                     // Step 2: Toss arm forward and shoot
    //                     // Step 3: Open clamp
    //                     elevator.moveCommand(targetPosition.raiseElevatorPosition)
    //                     .alongWith(Commands.defer(() -> {
    //                         if (elevator.getPosition() < targetPosition.raiseElevatorThreshold) {
    //                             return arm.moveCommand(-8d, true);
    //                         } else {
    //                             return arm.moveCommand(targetPosition.firstStageArmPosition, false)
    //                             .alongWith(intake.setVelocityCommand(-HandConstants.releaseVelocity))
    //                             .alongWith(
    //                                 Commands.waitUntil(() -> arm.getArmPosition() > targetPosition.firstStageArmThreshold)
    //                                 .andThen(hand.moveHandCommand(targetPosition.clamperJigglePosition))
    //                             );
    //                         }
    //                     }, Set.of(arm, intake, hand)))
    //                 );
    public Command scoreAlgaeNet() {
        return homeAll()
        .andThen(
            moveElevatorFirstStage(PositionsFactory.getAlgaeScoreBargePositions())
            .alongWith(
                moveArmSlow(-8d).until(() -> elevatorAboveThreshold(PositionsFactory.getAlgaeScoreBargePositions().raiseElevatorThreshold))
                .andThen(
                    moveArmFirstStage(PositionsFactory.getAlgaeScoreBargePositions())
                    .alongWith(
                        intake.releaseCommand()
                    )
                )
            )
        ).andThen(homeAll());
    }

    public Command initializeProcessorScore() {
        return homeAll().andThen(moveArmFirstStageSlow(PositionsFactory.getAlgaeHandoffPositions()));
    }

    public Command finishProcessorScore() {
        return intake.releaseCommand().alongWith(closeClamp());
    }
}