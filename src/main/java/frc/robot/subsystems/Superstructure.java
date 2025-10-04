package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.state.sequencer.positions.PositionConstants.CORAL_SCORE.*;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.NEW_ArmSubsystem;
import frc.robot.subsystems.elevator.NEW_ElevatorSubsystem;
import frc.robot.subsystems.hand.*;

public class Superstructure {
    private NEW_ArmSubsystem arm;
    private NEW_ElevatorSubsystem elevator;
    private NEW_HandClamperSubsystem hand;
    private NEW_HandIntakeSubsystem intake;

    public enum Level {
        L1,
        L2,
        L3,
        L4
    }

    public Superstructure(NEW_ArmSubsystem arm, NEW_ElevatorSubsystem elevator, NEW_HandClamperSubsystem hand, NEW_HandIntakeSubsystem intake) {
        this.arm = arm;
        this.elevator = elevator;
        this.hand = hand;
        this.intake = intake;
    }

    /**
     * IF ARM OUT: Move arm in then continue (Switching between levels)
     * MOVE ELEVATOR TO POSITION
     * IF ELEVATOR PAST THRESHOLD, MOVE ARM TO POSITION (SLOW FOR L1)
     */
    public Command goToCoralPositionCommand(Level level) {
        return new ConditionalCommand(
            arm.moveArmNormalSpeed(ArmConstants.armHomePosition),
            Commands.none(), 
            () -> arm.getArmPosition() > ArmConstants.willSmackReefThreshold
        ).until(() -> arm.getArmPosition() < ArmConstants.willSmackReefThreshold)
        .andThen(
            (switch (level) {
                case L1 -> elevator.moveElevatorCommand(CORAL_L1.raiseElevatorPosition);
                case L2 -> elevator.moveElevatorCommand(CORAL_L2.raiseElevatorPosition);
                case L3 -> elevator.moveElevatorCommand(CORAL_L3.raiseElevatorPosition);
                case L4 -> elevator.moveElevatorCommand(CORAL_L4.raiseElevatorPosition);
            }).alongWith(
                Commands.waitUntil(
                    switch (level) {
                        case L1 -> () -> elevator.getElevatorPosition() > CORAL_L1.raiseElevatorThresholdPosition;
                        case L2 -> () -> elevator.getElevatorPosition() > CORAL_L2.raiseElevatorThresholdPosition;
                        case L3 -> () -> elevator.getElevatorPosition() > CORAL_L3.raiseElevatorThresholdPosition;
                        case L4 -> () -> elevator.getElevatorPosition() > CORAL_L4.raiseElevatorThresholdPosition;
                    }
                ).andThen(
                    switch (level) {
                        case L1 -> arm.moveArmSlowSpeed(CORAL_L1.armForwardPosition);
                        case L2 -> arm.moveArmNormalSpeed(CORAL_L1.armForwardPosition);
                        case L3 -> arm.moveArmNormalSpeed(CORAL_L1.armForwardPosition);
                        case L4 -> arm.moveArmNormalSpeed(CORAL_L1.armForwardPosition);
                    }
                )
            )
        );
    }

    public Command goToAlgaePositionCommand() {
        return Commands.none();
    }

    public Command scoreCoralCommand() {
        return Commands.none();
    }

    public Command homeCommand() {
        return Commands.none();
    }

    public Command intakeCoralCommand() {
        return Commands.none();
    }

    public Command intakeAlgaeCommand() {
        return Commands.none();
    }
}