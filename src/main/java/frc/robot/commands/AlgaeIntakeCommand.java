package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.state.sequencer.Level;
import frc.robot.state.sequencer.SequenceManager;
import frc.robot.state.sequencer.positions.Positions;
import frc.robot.state.sequencer.positions.PositionsFactory;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.hand.HandClamperSubsystem;
import frc.robot.subsystems.hand.HandConstants;
import frc.robot.subsystems.hand.HandIntakeSubsystem;

public class AlgaeIntakeCommand extends Command {
    private final ElevatorSubsystem elevator;
    private final ArmSubsystem arm;
    private final HandClamperSubsystem clamp;
    private final HandIntakeSubsystem intake;
    private Positions position = PositionsFactory.getAlgaeFloorPickupPositions();

    public AlgaeIntakeCommand(ElevatorSubsystem elevator, ArmSubsystem arm, HandClamperSubsystem clamp, HandIntakeSubsystem intake) {
        this.elevator = elevator;
        this.arm = arm;
        this.clamp = clamp;
        this.intake = intake;
        addRequirements(elevator, arm, clamp, intake);
    }

    @Override
    public void initialize() {
        if(SequenceManager.getLevelSelection() == Level.L2) {
            position = PositionsFactory.getAlgaeReefL2PickupPositions();
        } else if(SequenceManager.getLevelSelection() == Level.L3) {
            position = PositionsFactory.getAlgaeReefL3PickupPositions();
        }
    }

    @Override
    public void execute() {
        elevator.moveElevatorNormalSpeed(position.raiseElevatorPosition);
        if(elevator.isAtPosition(position.raiseElevatorThreshold)) {
            arm.moveArmSlowSpeed(position.firstStageArmPosition);
            clamp.open(position.clamperIntakePosition);
            intake.intake(HandConstants.intakeAlgaeVelocity);
        }
    }

    @Override
    public void end(boolean interrupted) {
        arm.moveArmSlowSpeed(position.secondStageArmPosition);
        elevator.moveElevatorSlowSpeed(position.secondStageElevatorPosition);
        clamp.open(position.clamperHoldPosition);
        intake.stop();
    }
}