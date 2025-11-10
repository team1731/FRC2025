package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.state.sequencer.SequenceInput;
import frc.robot.state.sequencer.SequenceStateMachine;
import frc.robot.state.sequencer.SequenceManager;
import frc.robot.subsystems.arm.OLD_ArmSubsystem;
import frc.robot.subsystems.elevator.OLD_ElevatorSubsystem;
import frc.robot.subsystems.hand.OLD_HandClamperSubsystem;
import frc.robot.subsystems.hand.OLD_HandIntakeSubsystem;

public class AutoEndSequenceCommand extends Command {
    SequenceStateMachine m_scoreStateMachine;
    OLD_ElevatorSubsystem m_elevatorSubsystem;
    OLD_ArmSubsystem m_armSubsystem;
    OLD_HandClamperSubsystem m_clamperSubsystem;
    OLD_HandIntakeSubsystem m_intakeSubsystem;
    boolean m_sequenceStarted = false;
    boolean m_sequenceDone = false;

    public AutoEndSequenceCommand(OLD_ElevatorSubsystem elevatorSubsystem, OLD_ArmSubsystem armSubsystem,
            OLD_HandClamperSubsystem clamperSubsystem, OLD_HandIntakeSubsystem intakeSubsystem) {
        m_scoreStateMachine = SequenceManager.getStateMachine(elevatorSubsystem, armSubsystem, clamperSubsystem, intakeSubsystem);
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
