package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.hand.HandIntakeSubsystem;
import frc.robot.state.sequencer.GamePiece;
import frc.robot.subsystems.hand.HandClamperSubsystem;
import frc.robot.subsystems.hand.HandConstants;

public class IntakeCommand extends Command {
    HandClamperSubsystem m_handClamperSubsystem;
    HandIntakeSubsystem m_handIntakeSubsystem;
    GamePiece gamePiece;


    public IntakeCommand(HandClamperSubsystem handClamperSubsystem, HandIntakeSubsystem handIntakeSubsystem, GamePiece piece) {
        m_handClamperSubsystem = handClamperSubsystem;
        m_handIntakeSubsystem = handIntakeSubsystem;
        gamePiece = piece;
        addRequirements(m_handClamperSubsystem, handIntakeSubsystem);
    }

    @Override
    public void initialize() {
        m_handClamperSubsystem.open(gamePiece == GamePiece.CORAL? HandConstants.clamperCoralPosition : HandConstants.clamperAlgaePosition);
        m_handIntakeSubsystem.intake(HandConstants.intakeVelocity);

    }

    @Override
    public void end(boolean interrupted) {
        if(gamePiece == GamePiece.CORAL) {
            m_handClamperSubsystem.close();
        }
    }


}
