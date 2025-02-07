package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.hand.HandIntakeSubsystem;
import frc.robot.state.intake.IntakePositionConstants;
import frc.robot.subsystems.hand.HandClamperSubsystem;
import frc.robot.subsystems.hand.HandConstants;

public class AlgaeEjectCommand extends Command {
    HandClamperSubsystem m_handClamperSubsystem;
    HandIntakeSubsystem m_handIntakeSubsystem;


    public AlgaeEjectCommand(HandClamperSubsystem handClamperSubsystem, HandIntakeSubsystem handIntakeSubsystem) {
        m_handClamperSubsystem = handClamperSubsystem;
        m_handIntakeSubsystem = handIntakeSubsystem;
        addRequirements(m_handClamperSubsystem, handIntakeSubsystem);
    }

    @Override
    public void initialize() {
        m_handClamperSubsystem.close();
        m_handIntakeSubsystem.release(HandConstants.scoreAlgaeVelocity);

    }

    @Override
    public void end(boolean interrupted) {
       // m_handClamperSubsystem.close();
        m_handIntakeSubsystem.stop();
    }


}
