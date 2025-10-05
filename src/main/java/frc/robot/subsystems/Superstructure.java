package frc.robot.subsystems;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.state.sequencer.positions.*;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.NEW_ArmSubsystem;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.NEW_ElevatorSubsystem;
import frc.robot.subsystems.hand.*;

public class Superstructure {
    private NEW_ArmSubsystem arm;
    private NEW_ElevatorSubsystem elevator;
    private NEW_HandClamperSubsystem hand;
    private NEW_HandIntakeSubsystem intake;

    private Positions targetPosition = PositionsFactory.getCoralScoreL4Positions();
    private Level targetLevel = Level.L4;
    // private GamePiece targetPiece = GamePiece.CORAL;

    public enum GamePiece {
        ALGAE,
        CORAL
    }

    public enum Level {
        L1(PositionsFactory.getCoralScoreL1Positions(), PositionsFactory.getAlgaeFloorPickupPositions(), PositionsFactory.getAlgaeHandoffPositions()),
        L2(PositionsFactory.getCoralScoreL2Positions(), PositionsFactory.getAlgaeReefL2PickupPositions(), PositionsFactory.getAlgaeScoreBargePositions()),
        L3(PositionsFactory.getCoralScoreL3Positions(), PositionsFactory.getAlgaeReefL3PickupPositions(), PositionsFactory.getAlgaeScoreBargePositions()),
        L4(PositionsFactory.getCoralScoreL4Positions(), PositionsFactory.getAlgaeReefL3PickupPositions(), PositionsFactory.getAlgaeScoreBargePositions());

        public final Positions coralPositions, algaeIntakePositions, algaeScorePositions;
        private Level(Positions coral, Positions algaeIntake, Positions algaeScore) {
            this.coralPositions = coral;
            this.algaeIntakePositions = algaeIntake;
            this.algaeScorePositions = algaeScore;
        }
    }

    public Superstructure(NEW_ArmSubsystem arm, NEW_ElevatorSubsystem elevator, NEW_HandClamperSubsystem hand, NEW_HandIntakeSubsystem intake) {
        this.arm = arm;
        this.elevator = elevator;
        this.hand = hand;
        this.intake = intake;
    }

    public Command prepCoralScoreCommand() {
        return setPositionsCommand(targetLevel.coralPositions) // Set the positions based on target level
        .andThen(
            // If the arm is already out and the elevator is outside of the threshold, move arm to home position then move elevator
            Commands.either(
                arm.moveArmCommand(ArmConstants.armHomePosition),
                Commands.none(),  
                () -> arm.getArmPosition() > ArmConstants.willSmackReefThreshold && 
                    elevator.getElevatorPosition() > targetPosition.raiseElevatorPosition + 3.0 ||
                    elevator.getElevatorPosition() < targetPosition.raiseElevatorThreshold - 3.0
            )
        ).andThen(
            elevator.moveElevatorCommand(targetPosition.raiseElevatorPosition)
            .alongWith( // Move elevator to position
            // Move arm to position once the elevator is within the threshold
            Commands.waitUntil(() -> 
                elevator.getElevatorPosition() > targetPosition.raiseElevatorThreshold &&
                elevator.getElevatorPosition() < targetPosition.raiseElevatorPosition
            ).andThen(
                Commands.either(
                    arm.moveArmSlowCommand(targetPosition.firstStageArmPosition),
                    arm.moveArmCommand(targetPosition.firstStageArmPosition), 
                    () -> targetLevel == Level.L1
                )
            )
        ));
    }

    public Command finishCoralScoreCommand() {
        switch (targetLevel) {
            case L1:
                // OPEN CLAMP
                // GO TO HOME
                return hand.moveHandCommand(targetPosition.clamperOpenPosition).andThen(homeCommand());
            case L2:
                // MOVE ELEVATOR DOWN,
                // MOVE ARM TO SCORE POSITION,
                // OPEN CLAMP,
                // OUTTAKE CORAL
                return elevator.moveElevatorCommand(targetPosition.lowerElevatorThreshold)
                .alongWith(
                    arm.moveArmCommand(targetPosition.secondStageArmPosition),
                    hand.moveHandCommand(HandConstants.clamperCoralPosition),
                    intake.setVelocityCommand(-HandConstants.releaseVelocity)
                );
            default: // L3 and L4 same logic
                return arm.moveArmCommand(targetPosition.secondStageArmPosition)
                .andThen(
                    Commands.waitUntil(() -> arm.getArmPosition() > targetPosition.firstStageArmThreshold)
                    .andThen(
                        elevator.moveElevatorCommand(ElevatorConstants.elevatorHomePosition)
                        .alongWith(
                            Commands.waitUntil(() -> elevator.getElevatorPosition() < targetPosition.lowerElevatorThreshold)
                            .andThen(arm.moveArmCommand(ArmConstants.armHomePosition))
                        )
                    )
                );
        }
    }

    public Command prepAlgaeScoreCommand() {
        return setPositionsCommand(targetLevel.algaeScorePositions).andThen(
            switch (targetLevel) {
                case L1: // Processor score
                    yield clearArmCommand()
                    .andThen(elevator.moveElevatorCommand(ElevatorConstants.elevatorHomePosition))
                    .andThen(arm.moveArmCommand(targetPosition.firstStageArmPosition));
                default: // Net score
                    // first home
                    // Move arm all the way back
                    yield homeCommand()
                    .andThen(
                        // Step 1: Bring up elevator with arm back
                        // Step 2: Toss arm forward and shoot
                        // Step 3: Open clamp
                        elevator.moveElevatorCommand(targetPosition.raiseElevatorPosition)
                        .alongWith(Commands.defer(() -> {
                            if (elevator.getElevatorPosition() < targetPosition.raiseElevatorThreshold) {
                                return arm.moveArmSlowCommand(-8d);
                            } else {
                                return arm.moveArmCommand(targetPosition.firstStageArmPosition)
                                .alongWith(intake.setVelocityCommand(-HandConstants.releaseVelocity))
                                .alongWith(
                                    Commands.waitUntil(() -> arm.getArmPosition() > targetPosition.firstStageArmThreshold)
                                    .andThen(hand.moveHandCommand(targetPosition.clamperJigglePosition))
                                );
                            }
                        }, Set.of(arm)))
                    );
            }
        );
    }

    public Command finishAlgaeScoreCommand() {
        return setPositionsCommand(targetLevel.algaeScorePositions).andThen(
            switch (targetLevel) {
                case L1: // Processor score
                    yield intake.setVelocityCommand(-HandConstants.releaseVelocity)
                    .alongWith(hand.closeCommand());
                default: // Net score
                    yield elevator.moveElevatorCommand(ElevatorConstants.elevatorHomePosition)
                    .alongWith(Commands.waitUntil(() -> elevator.getElevatorPosition() < targetPosition.lowerElevatorThreshold)
                        .andThen(
                            arm.moveArmCommand(ArmConstants.armHomePosition)
                            .alongWith(
                                hand.closeCommand(),
                                intake.holdCommand()
                            )
                        )
                    );
            }
        );
    }

    public Command homeCommand() {
        return arm.moveArmCommand(ArmConstants.armHomePosition)
            .alongWith(
                elevator.moveElevatorCommand(ElevatorConstants.elevatorHomePosition),
                hand.moveHandCommand(HandConstants.clamperHomePosition)
            );
    }

    public Command intakeCoralCommand() {
        return setPositionsCommand(targetLevel.coralPositions).andThen(homeCommand().andThen(
            hand.moveHandCommand(HandConstants.clamperCoralPosition)
            .alongWith(intake.setVelocityCommand(HandConstants.intakeCoralVelocity))
        ));
    }

    public Command intakeAlgaeCommand() {
        return setPositionsCommand(targetLevel.algaeIntakePositions).andThen(
            switch (targetLevel) {
                case L1: // Floor pickup
                    // If arm out, bring arm back first
                    // Home command
                    // Move arm and hand to floor pickup position, run intake
                    yield clearArmCommand().andThen(homeCommand())
                    .andThen(
                        arm.moveArmCommand(targetPosition.firstStageArmPosition)
                        .alongWith(
                            Commands.waitUntil(() -> arm.getArmPosition() > targetPosition.firstStageArmThreshold)
                            .andThen(hand.moveHandCommand(HandConstants.clamperAlgaePosition).alongWith(
                                intake.setVelocityCommand(HandConstants.intakeAlgaeVelocity)
                            ))
                        )
                    );
                default: // Reef L2 and L3 both have the same sequence
                    // If arm out, bring arm back first
                    // If clamp out, bring clamp in first
                    // Move elevator up
                    // If elevator high enough, bring clamper out and spin intake
                    yield clearArmCommand()
                    .alongWith(clearHandCommand())
                    .andThen(
                        elevator.moveElevatorCommand(targetPosition.raiseElevatorPosition).alongWith(
                            Commands.waitUntil(() -> elevator.getElevatorPosition() > targetPosition.raiseElevatorThreshold &&
                                elevator.getElevatorPosition() < targetPosition.raiseElevatorPosition + 3.0
                            )
                            .andThen(
                                hand.moveHandCommand(targetPosition.clamperOpenPosition)
                                .alongWith(intake.setVelocityCommand(HandConstants.intakeAlgaeVelocity))
                            )
                        )
                    );
            }
        );
    }

    public Command finishAlgaeIntakeCommand() {
        return setPositionsCommand(targetLevel.algaeIntakePositions).andThen(
            switch (targetLevel) {
                case L1: // Finish floor pickup
                    yield arm.moveArmCommand(targetPosition.secondStageArmPosition)
                    .deadlineFor(intake.setVelocityCommand(HandConstants.intakeAlgaeVelocity))
                    .andThen(intake.holdCommand());
                default: // Finish reef L2 and L3 pickup
                    yield elevator.moveElevatorCommand(targetPosition.secondStageElevatorPosition)
                    .andThen(hand.moveHandCommand(HandConstants.clamperAlgaePosition))
                    .andThen(elevator.moveElevatorCommand(ElevatorConstants.elevatorHomePosition))
                    .andThen(arm.moveArmCommand(targetPosition.secondStageArmPosition));
            }
        );
    }

    // ===========================
    //       Helper Commands
    // ===========================


    public Command setLevelCommand(Level level) {
        return new InstantCommand(() -> targetLevel = level);
    }

    // public Command setGamePieceCommand(GamePiece piece) {
    //     return new InstantCommand(() -> targetPiece = piece);
    // }

    private Command setPositionsCommand(Positions positions) {
        return new InstantCommand(() -> targetPosition = positions);
    }


    private Command clearArmCommand() {
        return Commands.either(
            arm.moveArmCommand(ArmConstants.armHomePosition),
            Commands.none(),
            () -> arm.getArmPosition() > ArmConstants.willSmackReefThreshold && 
                elevator.getElevatorPosition() > targetPosition.raiseElevatorPosition + 3.0 ||
                elevator.getElevatorPosition() < targetPosition.raiseElevatorThreshold - 3.0
        );
    }

    private Command clearHandCommand() {
        return Commands.either(
            hand.moveHandCommand(HandConstants.clamperHomePosition),
            Commands.none(),
            () -> hand.getPosition() > 0.02 &&
                elevator.getElevatorPosition() > targetPosition.raiseElevatorPosition + 3.0 ||
                elevator.getElevatorPosition() < targetPosition.raiseElevatorThreshold - 3.0
        );
    }
}