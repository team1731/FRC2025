package frc.robot.commands;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.hand.HandIntakeSubsystem;
import frc.robot.subsystems.hand.HandClamperSubsystem;

public class CoralIntakeCommand extends Command {
    HandClamperSubsystem m_handClamperSubsystem;
    HandIntakeSubsystem m_handIntakeSubsystem;


    public CoralIntakeCommand(HandClamperSubsystem handClamperSubsystem, HandIntakeSubsystem handIntakeSubsystem) {
        m_handClamperSubsystem = handClamperSubsystem;
        m_handIntakeSubsystem = handIntakeSubsystem;
        addRequirements(m_handClamperSubsystem, handIntakeSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("int coral intake command!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
        m_handClamperSubsystem.open(4);
        m_handIntakeSubsystem.intake(5000/60);

    }

    @Override
    public void end(boolean interrupted) {
        m_handClamperSubsystem.close();
        m_handIntakeSubsystem.stop();
        
        
    }


}
