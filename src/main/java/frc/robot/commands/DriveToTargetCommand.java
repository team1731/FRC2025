package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Robot;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.AprilTagSubsystem;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.camera.CameraChoice;
import frc.robot.subsystems.vision.helpers.AprilTagTargetTracker;

public class DriveToTargetCommand extends Command {
    private CommandSwerveDrivetrain m_driveSubsystem;
    private CommandXboxController m_xboxController;
    private AprilTagTargetTracker aprilTagTargetTracker;
    private boolean m_commandDone = false;
    private CameraChoice m_cameraChoice = CameraChoice.BatSide;
    private double fieldCentricX;
    private double fieldCentricY;
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond)/2;   //Drive at 1/5th of the max speed!!!!!!!!!

    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.05).withRotationalDeadband(MaxAngularRate * 0.05) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final SwerveRequest.FieldCentricFacingAngle driveAtLocation =  new SwerveRequest.FieldCentricFacingAngle().withRotationalDeadband( VisionConstants.MAX_ANGULAR_SPEED * 0.01) // Add a 10% deadband
		.withDriveRequestType(DriveRequestType.OpenLoopVoltage).withDeadband((MaxSpeed * 0.05));

    public DriveToTargetCommand(CommandSwerveDrivetrain driveSubsystem, CommandXboxController xboxController, CameraChoice cameraChoice) {
        m_driveSubsystem = driveSubsystem;
        m_xboxController = xboxController;
        m_cameraChoice = cameraChoice;

        addRequirements(m_driveSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("DriveToTargetCommand: initializing...");
        driveAtLocation.HeadingController.setPID(10,0,0);
	    driveAtLocation.HeadingController.enableContinuousInput(-Math.PI/2, Math.PI/2);
        m_commandDone = false;
        AprilTagSubsystem aprilTagSubsystem = m_driveSubsystem.getAprilTagSubsystem();
        if(aprilTagSubsystem != null) {
            aprilTagTargetTracker = new AprilTagTargetTracker(aprilTagSubsystem.getCamera(m_cameraChoice), null);
        } else {
            m_commandDone = true;
        }
    }

    @Override
    public void execute() {
        fieldCentricX = (Math.abs(m_xboxController.getLeftY()) * m_xboxController.getLeftY());
        fieldCentricY = (Math.abs(m_xboxController.getLeftX()) * m_xboxController.getLeftX());

        if (Robot.isRedAlliance()) {
            fieldCentricX = fieldCentricX * -1;
            fieldCentricY = fieldCentricY * -1;

        }
        
        SmartDashboard.putNumber ("fieldCentricX", fieldCentricX);
        SmartDashboard.putNumber ("fieldCentricY",fieldCentricY);

        aprilTagTargetTracker.recalculateDriveFeedback(m_driveSubsystem.getCurrentPose(), fieldCentricX, fieldCentricY);
        m_driveSubsystem.setControl(

            driveAtLocation.withVelocityX(aprilTagTargetTracker.getCalculatedX() * MaxSpeed)                                                                                                                     
                .withVelocityY(aprilTagTargetTracker.getCalculatedY()* MaxSpeed) 
                .withTargetDirection(aprilTagTargetTracker.getRotationTarget())
        );

        SmartDashboard.putNumber("PID Setpoint", driveAtLocation.HeadingController.getSetpoint());
		SmartDashboard.putNumber("PID Output", driveAtLocation.HeadingController.getLastAppliedOutput());
		SmartDashboard.putNumber("PID Error", driveAtLocation.HeadingController.getPositionError());
        SmartDashboard.putNumber ("current heading",m_driveSubsystem.getState().Pose.getRotation().getDegrees());
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
