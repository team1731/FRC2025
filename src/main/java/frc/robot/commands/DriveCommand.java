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
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.helpers.AprilTagTargetTracker;

public class DriveCommand extends Command {
    public enum DriveMode {
        DEFAULT, TARGETING
    }

    private CommandSwerveDrivetrain m_driveSubsystem;
    private CommandXboxController m_xboxController;
    private static AprilTagTargetTracker aprilTagTargetTracker;
    private static DriveMode currentDriveMode = DriveMode.DEFAULT; 
    private static boolean lockedOnce = false;

    /*
     * Default drive state
     */
    private double DefaultMaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double DefaultMaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    private final SwerveRequest.FieldCentric defaultDrive = new SwerveRequest.FieldCentric()
      .withDeadband(DefaultMaxSpeed * 0.05).withRotationalDeadband(DefaultMaxAngularRate * 0.05); // Add a 10% deadband

    /*
     * Drive to target state
     */
    private double fieldCentricX;
    private double fieldCentricY;
    private double DriveToTargetMaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);   
    private final SwerveRequest.FieldCentricFacingAngle driveAtTarget =  new SwerveRequest.FieldCentricFacingAngle().withRotationalDeadband(VisionConstants.MAX_ANGULAR_SPEED * 0.01) // Add a 10% deadband
		.withDriveRequestType(DriveRequestType.OpenLoopVoltage).withDeadband((DriveToTargetMaxSpeed * 0.05));

    private int lostTargetCount = 0;
    
    public DriveCommand(CommandSwerveDrivetrain driveSubsystem, CommandXboxController xboxController) {
        m_driveSubsystem = driveSubsystem;
        m_xboxController = xboxController;
        addRequirements(m_driveSubsystem);
    }

    public static void setDriveMode(DriveMode driveMode) {
        System.out.println("DriveCommand: changing drive mode to " + driveMode);
        currentDriveMode = driveMode;
        lockedOnce = false;

    }

    @Override
    public void initialize() {
        System.out.println("DriveCommand: initializing...");
        driveAtTarget.HeadingController.setPID(10,0,0);
	    driveAtTarget.HeadingController.enableContinuousInput(-Math.PI/2, Math.PI/2);
        aprilTagTargetTracker = AprilTagTargetTracker.getTargetTracker(m_driveSubsystem.getAprilTagSubsystem());
    }

    @Override
    public void execute() {
        if(currentDriveMode == DriveMode.DEFAULT) {
            drive();
        } else {
            driveToTarget();
        }
    }

    private void drive() {
        m_driveSubsystem.setControl(
          defaultDrive.withVelocityX(-(Math.abs(m_xboxController.getLeftY()) * m_xboxController.getLeftY()) * DefaultMaxSpeed)                                                                                                                     
              .withVelocityY(-(Math.abs(m_xboxController.getLeftX()) * m_xboxController.getLeftX()) * DefaultMaxSpeed) 
              .withRotationalRate(-m_xboxController.getRightX() * DefaultMaxAngularRate)
        );
    }

    private void driveToTarget() {
        fieldCentricX = (Math.abs(m_xboxController.getLeftY()) * m_xboxController.getLeftY());
        fieldCentricY = (Math.abs(m_xboxController.getLeftX()) * m_xboxController.getLeftX());

        if (Robot.isRedAlliance()) {
            fieldCentricX = fieldCentricX * -1;
            fieldCentricY = fieldCentricY * -1;

        }
        
        SmartDashboard.putNumber ("fieldCentricX", fieldCentricX);
        SmartDashboard.putNumber("fieldCentricY", fieldCentricY);
        aprilTagTargetTracker.recalculateDriveFeedback(m_driveSubsystem.getCurrentPose(), fieldCentricX, fieldCentricY);
        SmartDashboard.putBoolean("hasVisibleTarget", aprilTagTargetTracker.hasVisibleTarget());

        if (aprilTagTargetTracker.hasVisibleTarget()) {
            lostTargetCount = 0;
            lockedOnce = true;
        } else {
            lostTargetCount++;
        }

        if (aprilTagTargetTracker.hasVisibleTarget() || ((lostTargetCount < 5) && lockedOnce == true)) {

            m_driveSubsystem.setControl(
                    driveAtTarget.withVelocityX(aprilTagTargetTracker.getCalculatedX() * DriveToTargetMaxSpeed)
                            .withVelocityY(aprilTagTargetTracker.getCalculatedY() * DriveToTargetMaxSpeed)
                            .withTargetDirection(aprilTagTargetTracker.getRotationTarget()));
        } else {
            m_driveSubsystem.setControl(
                    defaultDrive
                            .withVelocityX(-(Math.abs(m_xboxController.getLeftY()) * m_xboxController.getLeftY())
                                    * DefaultMaxSpeed)
                            .withVelocityY(-(Math.abs(m_xboxController.getLeftX()) * m_xboxController.getLeftX())
                                    * DefaultMaxSpeed)
                            .withRotationalRate(-m_xboxController.getRightX() * DefaultMaxAngularRate));
        }

        SmartDashboard.putNumber("PID Setpoint", driveAtTarget.HeadingController.getSetpoint());
        SmartDashboard.putNumber("PID Output", driveAtTarget.HeadingController.getLastAppliedOutput());
		SmartDashboard.putNumber("PID Error", driveAtTarget.HeadingController.getPositionError());
        SmartDashboard.putNumber ("current heading",m_driveSubsystem.getState().Pose.getRotation().getDegrees());
    }
}
