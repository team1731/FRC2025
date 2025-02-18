package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.state.sequencer.SequenceInput;
import frc.robot.state.sequencer.SequenceStateMachine;
import frc.robot.state.sequencer.Sequence;
import frc.robot.state.sequencer.SequenceManager;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.hand.HandClamperSubsystem;
import frc.robot.subsystems.hand.HandIntakeSubsystem;

public class AutoEndSequenceCommand extends Command {
    SequenceStateMachine m_scoreStateMachine;
    ElevatorSubsystem m_elevatorSubsystem;
    ArmSubsystem m_armSubsystem;
    HandClamperSubsystem m_clamperSubsystem;
    HandIntakeSubsystem m_intakeSubsystem;
    boolean m_sequenceStarted = false;
    boolean m_sequenceDone = false;

    public AutoEndSequenceCommand(ElevatorSubsystem elevatorSubsystem, ArmSubsystem armSubsystem,
            HandClamperSubsystem clamperSubsystem, HandIntakeSubsystem intakeSubsystem) {
        m_scoreStateMachine = SequenceManager.getStateMachine(elevatorSubsystem, armSubsystem, clamperSubsystem,
                intakeSubsystem);
        m_elevatorSubsystem = elevatorSubsystem;
        m_armSubsystem = armSubsystem;
        m_clamperSubsystem = clamperSubsystem;
        m_intakeSubsystem = intakeSubsystem;
        addRequirements(m_elevatorSubsystem, m_armSubsystem, m_clamperSubsystem, m_intakeSubsystem);
    }

    @Override
    public void initialize() {
        // Just the act of adding the requirements will kill other commands that
        m_scoreStateMachine.setInput(SequenceInput.BUTTON_RELEASED);
        System.out.println("Setting input to SCORE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1");

    }

    @Override
    public void execute() {
        // do nothing
    }

    @Override
    public void end(boolean interrupted) {
        // do nothing
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
