package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.state.score.ScoreStateMachine;
import frc.robot.subsystems.hand.HandClamperSubsystem;
import frc.robot.subsystems.hand.HandIntakeSubsystem;

public class ResetHandCommand extends Command {
    ScoreStateMachine m_scoreStateMachine;
    HandClamperSubsystem m_clamperSubsystem;
    HandIntakeSubsystem m_intakeSubsystem;

    public ResetHandCommand(HandClamperSubsystem clamperSubsystem, HandIntakeSubsystem intakeSubsystem) {
        m_clamperSubsystem = clamperSubsystem;
        m_intakeSubsystem = intakeSubsystem;
        addRequirements(m_clamperSubsystem, m_intakeSubsystem);
    }

    @Override
    public void initialize() {
        m_clamperSubsystem.close();
        m_intakeSubsystem.stop();
    }
}

