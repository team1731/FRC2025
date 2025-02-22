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
 * This command is a fire-and-foreget variant on the RunSequenceCommand intended for autonomous
 * It fires off the state machine but doesn't wait for it to finish.
 */

public class AutoFireSequenceCommand extends Command {
    SequenceStateMachine m_scoreStateMachine;
    ElevatorSubsystem m_elevatorSubsystem;
    ArmSubsystem m_armSubsystem;
    HandClamperSubsystem m_clamperSubsystem;
    HandIntakeSubsystem m_intakeSubsystem;
    boolean m_sequenceStarted = false;
    boolean m_commandDone = false;

    public AutoFireSequenceCommand(ElevatorSubsystem elevatorSubsystem, ArmSubsystem armSubsystem, HandClamperSubsystem clamperSubsystem, HandIntakeSubsystem intakeSubsystem) {
        m_scoreStateMachine = SequenceManager.getStateMachine();
        m_elevatorSubsystem = elevatorSubsystem;
        m_armSubsystem = armSubsystem;
        m_clamperSubsystem = clamperSubsystem;
        m_intakeSubsystem = intakeSubsystem;
        addRequirements(m_elevatorSubsystem, m_armSubsystem, m_clamperSubsystem, m_intakeSubsystem);
    }

    public void runStateMachine() {
        System.out.println("AutoFireSequenceCommand: running sequence state machine");
        m_sequenceStarted = true;
        Sequence sequence = SequenceManager.getSequence();
        if(sequence == null) {
            System.out.println("FireSequenceCommand: ERROR - SEQUENCE NOT FOUND - check auto command configuration, exiting command");
            m_commandDone = true;
            return;
        }

        System.out.println("AutoFireSequenceCommand: Sequence chosen " + sequence);
        m_scoreStateMachine.setSequence(sequence);
        m_scoreStateMachine.setInput(SequenceInput.BEGIN);

        // This command is fire and forget, so we are done
        System.out.println("AutoFireSequenceCommand: fired off state machine, telling command to exit");
        m_commandDone = true;
    }

    @Override
    public void initialize() {
        m_sequenceStarted = false;
        m_commandDone = false;
        if(m_scoreStateMachine.isReady()) {
            runStateMachine();
        }
    }

    @Override
    public void execute() {
        if(!m_sequenceStarted && m_scoreStateMachine.isReady()) {
            runStateMachine();
        }
    }

    @Override
    public void end(boolean interrupted) {
        if(!m_commandDone) {
            System.out.println("AutoFireSequenceCommand: command interrupted");
            m_commandDone = true;
        }
    }

    @Override
    public boolean isFinished() {
        return m_commandDone;
    }
}
