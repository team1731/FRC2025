package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.hand.HandIntakeSubsystem;
import frc.robot.subsystems.hand.HandSubsystem;

public class CoralIntakeCommand extends Command {
    HandSubsystem m_handSubsystem;
    HandIntakeSubsystem m_handIntakeSubsystem;
    boolean isFinished = true;

    public CoralIntakeCommand(HandSubsystem handSubsystem, HandIntakeSubsystem handIntakeSubsystem) {
        m_handSubsystem = handSubsystem;
        m_handIntakeSubsystem = handIntakeSubsystem;
        addRequirements(handSubsystem, handIntakeSubsystem);
    }

    @Override
    public void initialize() {
        m_handSubsystem.open(0);
        m_handIntakeSubsystem.intake(0);
    }

    @Override
    public void end(boolean interrupted) {
        m_handSubsystem.close();
        m_handIntakeSubsystem.stop();
        isFinished = true;
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }
}
