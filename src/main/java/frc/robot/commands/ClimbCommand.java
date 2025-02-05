package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.state.intake.IntakePositionConstants;
import frc.robot.subsystems.climb.ClimbConstants;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.hand.HandConstants;

public class ClimbCommand extends Command {
    
    ClimbSubsystem m_climbSubsystem;

    public ClimbCommand(ClimbSubsystem climbSubsystem) {
        m_climbSubsystem = climbSubsystem;
        addRequirements(m_climbSubsystem); //why is this needed
    }

    @Override
    public void initialize() {
        m_climbSubsystem.moveClimb(ClimbConstants.maxClimbPosition);
    }

    @Override
    public void end(boolean interrupted) {
        m_climbSubsystem.stopClimb();
    }
}
