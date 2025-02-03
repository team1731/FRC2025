package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.hand.HandIntakeSubsystem;
import frc.robot.subsystems.hand.HandClamperSubsystem;

public class CoralIntakeCommand extends Command {
    HandClamperSubsystem m_handClamperSubsystem;
    HandIntakeSubsystem m_handIntakeSubsystem;
    boolean isFinished = true;

    public CoralIntakeCommand(HandClamperSubsystem handClamperSubsystem, HandIntakeSubsystem handIntakeSubsystem) {
        m_handClamperSubsystem = handClamperSubsystem;
        m_handIntakeSubsystem = handIntakeSubsystem;
        addRequirements(m_handClamperSubsystem, handIntakeSubsystem);
    }

    @Override
    public void initialize() {
        m_handClamperSubsystem.open(0);
        m_handIntakeSubsystem.intake(0);
    }

    @Override
    public void end(boolean interrupted) {
        m_handClamperSubsystem.close();
        m_handIntakeSubsystem.stop();
        isFinished = true;
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }
}
