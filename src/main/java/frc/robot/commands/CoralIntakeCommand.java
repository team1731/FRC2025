package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.state.sequencer.positions.PositionsFactory;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.hand.HandClamperSubsystem;
import frc.robot.subsystems.hand.HandConstants;
import frc.robot.subsystems.hand.HandIntakeSubsystem;

public class CoralIntakeCommand extends Command {

    private ArmSubsystem arm;
    private HandClamperSubsystem clamp;
    private HandIntakeSubsystem intake;
    private ElevatorSubsystem elevator;

    public CoralIntakeCommand(ArmSubsystem arm, HandClamperSubsystem clamp, HandIntakeSubsystem intake, ElevatorSubsystem elevator) {
        this.arm = arm;
        this.clamp = clamp;
        this.intake = intake;
        this.elevator = elevator;
        addRequirements(arm, clamp, intake, elevator);
    }

    @Override
    public void initialize() {
        arm.moveArmNormalSpeed(PositionsFactory.getCoralFeederPickupPositions().firstStageArmPosition);
        clamp.open(PositionsFactory.getCoralFeederPickupPositions().clamperIntakePosition);
        intake.intake(HandConstants.intakeCoralVelocity);
        elevator.moveElevatorNormalSpeed(ElevatorConstants.elevatorHomePosition);
    }

    @Override
    public void end(boolean interrupted) {
        arm.moveArmSlowSpeed(ArmConstants.armHomePosition);
        clamp.close();
        intake.stop();
    }
}
