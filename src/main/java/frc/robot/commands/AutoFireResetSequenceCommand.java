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

/*
 * This command is a fire-and-foreget variant on the ResetSequenceCommand intended for autonomous
 * It fires off the state machine but doesn't wait for it to finish.
 */

public class AutoFireResetSequenceCommand extends Command {
    SequenceStateMachine m_scoreStateMachine;
    ElevatorSubsystem m_elevatorSubsystem;
    ArmSubsystem m_armSubsystem;
    HandClamperSubsystem m_clamperSubsystem;
    HandIntakeSubsystem m_intakeSubsystem;
    boolean m_commandDone = false;

    public AutoFireResetSequenceCommand(ElevatorSubsystem elevatorSubsystem, ArmSubsystem armSubsystem, HandClamperSubsystem clamperSubsystem, HandIntakeSubsystem intakeSubsystem) {
        m_scoreStateMachine = SequenceManager.getStateMachine();
        m_elevatorSubsystem = elevatorSubsystem;
        m_armSubsystem = armSubsystem;
        m_clamperSubsystem = clamperSubsystem;
        m_intakeSubsystem = intakeSubsystem;
        addRequirements(m_elevatorSubsystem, m_armSubsystem, m_clamperSubsystem, m_intakeSubsystem);
    }

    @Override
    public void initialize() {
        m_commandDone = false;
        System.out.println("AutoFireResetSequenceCommand: running reset sequence...");
        m_scoreStateMachine.setSequence(Sequence.RESET);
        m_scoreStateMachine.setInput(SequenceInput.BEGIN);
        m_commandDone = true;
    }

    @Override
    public void end(boolean interrupted) {
        if(!m_commandDone) {
            System.out.println("AutoFireResetSequenceCommand: command interrupted");
            m_commandDone = true;
        }
    }

    @Override
    public boolean isFinished() {
        return m_commandDone;
    }
}
