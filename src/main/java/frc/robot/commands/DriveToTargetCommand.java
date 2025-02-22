package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.AprilTagSubsystem;
import frc.robot.subsystems.vision.helpers.AprilTagTargetTracker;

public class DriveToTargetCommand extends Command {
    private CommandSwerveDrivetrain m_driveSubsystem;
    private CommandXboxController m_xboxController;
    private AprilTagTargetTracker aprilTagTargetTracker;
    private boolean m_commandDone = false;
    private double fieldCentricX;
    private double fieldCentricY;
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    public DriveToTargetCommand(CommandSwerveDrivetrain driveSubsystem, CommandXboxController xboxController) {
        m_driveSubsystem = driveSubsystem;
        m_xboxController = xboxController;
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
            fieldCentricX = -(Math.abs(m_xboxController.getLeftY()) * m_xboxController.getLeftY()) * MaxSpeed;
            fieldCentricY = -(Math.abs(m_xboxController.getLeftX()) * m_xboxController.getLeftX()) * MaxSpeed;

            aprilTagTargetTracker.recalculateDriveFeedback(m_driveSubsystem.getCurrentPose());
            if(aprilTagTargetTracker.hasVisibleTarget()) {
                m_driveSubsystem.applyRequest(
                    () -> drive.withVelocityX(fieldCentricX)                                                                                                                     
                        .withVelocityY(aprilTagTargetTracker.getCalculatedStrafe()) 
                        .withRotationalRate(aprilTagTargetTracker.getCalcuatedTurn())
                );
            } else {
                m_driveSubsystem.applyRequest(
                    () -> drive.withVelocityX(-(Math.abs(m_xboxController.getLeftY()) * m_xboxController.getLeftY()) * MaxSpeed)                                                                                                                     
                        .withVelocityY(-(Math.abs(m_xboxController.getLeftX()) * m_xboxController.getLeftX()) * MaxSpeed) 
                        .withRotationalRate(-m_xboxController.getRightX() * MaxAngularRate)
            
                );
            }
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
