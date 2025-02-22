package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.AprilTagSubsystem;
import frc.robot.subsystems.vision.helpers.AprilTagTargetTracker;

public class DriveToTargetCommand extends Command {
    private CommandSwerveDrivetrain m_driveSubsystem;
    private AprilTagTargetTracker aprilTagTargetTracker;
    private boolean m_commandDone = false;

    public DriveToTargetCommand(CommandSwerveDrivetrain driveSubsystem) {
        m_driveSubsystem = driveSubsystem;
    }

    @Override
    public void initialize() {
        m_commandDone = false;
        AprilTagSubsystem aprilTagSubsystem = m_driveSubsystem.getAprilTagSubsystem();
        if(aprilTagSubsystem != null) {
            aprilTagTargetTracker = new AprilTagTargetTracker(aprilTagSubsystem.getCamera1(), aprilTagSubsystem.getCamera2());
        } else {
            m_commandDone = true;
        }
    }

    @Override
    public void execute() {
        if(aprilTagTargetTracker != null) {
            // TODO IMPLEMENT
        }
    }

    @Override
    public void end(boolean interrupted) {
        if(!m_commandDone) {
            System.out.println("DriveToTargetCommand: command interrupted");
            m_commandDone = true;
        }
    }

    @Override
    public boolean isFinished() {
        return m_commandDone;
    }
}
