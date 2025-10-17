package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climb.ClimbConstants;
import frc.robot.subsystems.climb.OLD_ClimbSubsystem;

public class ClimbReadyCommand extends Command {
    
    OLD_ClimbSubsystem m_climbSubsystem;

    public ClimbReadyCommand(OLD_ClimbSubsystem climbSubsystem) {
        m_climbSubsystem = climbSubsystem;
        addRequirements(m_climbSubsystem);
    }

    @Override
    public void initialize() {
        m_climbSubsystem.moveClimb(ClimbConstants.climbReadyPosition);
    }

    @Override
    public void end(boolean interrupted) {
        m_climbSubsystem.stopClimb();
    }
}
