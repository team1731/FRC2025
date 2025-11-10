package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.frc1731.subsystem.BaseSubsystem;
import frc.robot.AutoLoader;
import frc.robot.Robot;
import frc.robot.subsystems.drive.generated.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.generated.TunerConstants;
import frc.robot.subsystems.vision.AprilTagSubsystem;
import frc.robot.subsystems.vision.VSLAMSubsystem;
import frc.robot.subsystems.vision.helpers.AprilTagTargetTracker;

import static frc.robot.subsystems.drive.SwerveConstants.*;

public class SwerveSubsystem extends BaseSubsystem {
    private CommandSwerveDrivetrain drivetrain;
    private AprilTagSubsystem aprilTagSubsystem;
    private VSLAMSubsystem vslamSubsystem;

    private ChassisSpeeds targetSpeeds = new ChassisSpeeds();

    private AprilTagTargetTracker aprilTagTargetTracker;

    private double fieldCentricHeading = 0.0;
    private double robotCentricHeading = 0.0;

    private double lostTargetCount = 0;
    private boolean lockedOnce = true;

    private final Telemetry telemetry = new Telemetry(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond));

    private static Trigger isRobotEnabled = new Trigger(() -> DriverStation.isEnabled());

    private DrivetrainVisionCallback visionCallback = (Pose2d pose, double timestamp, Matrix<N3,N1> visionMeasurementStdDevs) -> {
        drivetrain.addVisionMeasurement(pose, timestamp, visionMeasurementStdDevs);  // comment this out to disable vslam
    };

    public SwerveSubsystem(boolean enabled) {
        super(enabled);
        if(!enabled) return;

        drivetrain = TunerConstants.createDrivetrain();
        driveAtTarget.HeadingController.setPID(10,0,0);
	    driveAtTarget.HeadingController.enableContinuousInput(-Math.PI/2, Math.PI/2);

        if(kUseAprilTags) {
            aprilTagSubsystem = new AprilTagSubsystem(true);
            aprilTagTargetTracker = AprilTagTargetTracker.getTargetTracker(aprilTagSubsystem);
            aprilTagSubsystem.startAutoLineup();
        }

        if (kUseVSLAM) {
            vslamSubsystem = new VSLAMSubsystem(visionCallback);
            vslamSubsystem.configure();
        }

        configureAutoBuilder();
        drivetrain.registerTelemetry(telemetry::telemeterize);

        isRobotEnabled.whileTrue(new InstantCommand(() -> aprilTagSubsystem.stopAutoLineup()))
        .onFalse(new InstantCommand(() -> aprilTagSubsystem.startAutoLineup()));
    }

    public void resetPose(Pose2d pose) {
        drivetrain.resetPose(pose);
        if(kUseVSLAM) {
            vslamSubsystem.calculateNewOffset(pose);
        }
    }

    public AprilTagSubsystem getAprilTagSubsystem() {
        return kUseAprilTags? aprilTagSubsystem : null;
    }
    public VSLAMSubsystem getVSLAMSubsystem() {
        return kUseVSLAM? vslamSubsystem : null;
    }

    public Pose2d getCurrentPose() {
        return drivetrain.getState().Pose;
    }

    public SwerveDriveState getState() {
        return drivetrain.getState();
    }

    public ChassisSpeeds getWheelSpeeds() {
        return getState().Speeds;
    }

    /*
     * This method will get called in two instances:
     * 1. After the VSLAM connects successfully
     * 2. When the alliance changes
     */
    public void configureInitialPosition() {
        System.out.println("CommandSwerveDrivetrain: configuring a new position");

	    Pose2d startingConfiguration = Robot.isRedAlliance()? 
            new Pose2d(10.38, 3.01, new Rotation2d(0)) : 
            new Pose2d(7.168, 5.006, new Rotation2d(Math.toRadians(180)));
        resetPose(startingConfiguration);

        Rotation2d operatorPerspective = Robot.isRedAlliance()? 
                new Rotation2d(Math.toRadians(180)) : 
                new Rotation2d(Math.toRadians(0));
        drivetrain.setOperatorPerspectiveForward(operatorPerspective);
    }

    public void configureAutoBuilder() {
        try {
            AutoBuilder.configure(
                this::getCurrentPose,
                this::resetAutoPose,
                this::getWheelSpeeds,
                // Consumer of ChassisSpeeds to drive the robot
                (speeds, feedsforwards)->drivetrain.setControl(autoRequest.withSpeeds(speeds)),
                new PPHolonomicDriveController(
                    kPPConstants,
                    kPPConstants
                ), RobotConfig.fromGUISettings(),
                () -> AutoLoader.flipForRed(),
                this);
        } catch(Exception e) {
            System.out.println("CommandSwerveDrivetrain error - failed to configure auto bindings");
        }
    }

    public void zeroHeading() {
        vslamSubsystem.zeroHeading();
    }

    /**
     * Only reset the pose of the robot is we are in simulation, otherwise the VSLAM will read current position
     */
    private void resetAutoPose(Pose2d pose) {
        if (Robot.isSimulation() || !kUseVSLAM || (kUseVSLAM && !vslamSubsystem.isConnected())) {
            this.resetPose(pose);
        }
    }

    @Override
    public void periodicTelemetry() {
        drivetrain.periodic();

        logger.log("Current Pose", getCurrentPose());
        logger.log("Current Speeds", getWheelSpeeds());
        logger.log("Target Speeds", targetSpeeds);

        if(kUseVSLAM) {
            logger.log("VSLAM Connected", vslamSubsystem.isConnected());
            logger.log("VSLAM Tracking", vslamSubsystem.isTracking());

            vslamSubsystem.cleanUpSubroutineMessages();
            vslamSubsystem.processHeartbeat();
        }

        Robot.kFieldLayout.setSimulatedRobotPose(getCurrentPose());
    }

    public Command drive(CommandXboxController m_xboxController, BooleanSupplier isFieldCentric) {
        return run(() -> {
            Translation2d RotationCenter =  new Translation2d();

            if(m_xboxController.getHID().getLeftStickButton()){ // might need to flip X and Y due to field begin Y,X
                fieldCentricHeading = Math.toDegrees(Math.atan2(m_xboxController.getLeftX(),  m_xboxController.getLeftY())); // desired heading in field centric
                robotCentricHeading = drivetrain.getState().Pose.getRotation().getDegrees() - fieldCentricHeading; // current robot rotation in degrees
                if(robotCentricHeading >= 0 && robotCentricHeading < 90){ // between 0 and 90
                    RotationCenter = new Translation2d( 0.3, 0.3);
                } else if(robotCentricHeading >= 90 && robotCentricHeading < 180){ // between 90 and 180
                    RotationCenter = new Translation2d( 0.3, -0.3);
                } else if(robotCentricHeading >= 180 && robotCentricHeading < 270){ // between 180 and 270
                    RotationCenter = new Translation2d( -0.3, -0.3);
                } else if(robotCentricHeading >= 270 && robotCentricHeading < 0){ // between 270 and 360
                    RotationCenter = new Translation2d( -0.3, 0.3);
                } else {
                    RotationCenter = new Translation2d(0, 0);
                }
            } else {
                RotationCenter = new Translation2d(0, 0);
            }

            if ((Math.abs(m_xboxController.getLeftY()) < 0.05) && 
                (Math.abs(m_xboxController.getLeftX()) < 0.05) &&
                (Math.abs(m_xboxController.getRightX()) < 0.05)){
                    drivetrain.setControl(brake);
            } else {
                if (isFieldCentric.getAsBoolean()) {
                    drivetrain.setControl(
                        kFieldCentricControl
                            .withVelocityX(-(Math.abs(m_xboxController.getLeftY()) * m_xboxController.getLeftY()) * kMaxSpeed)
                            .withVelocityY(-(Math.abs(m_xboxController.getLeftX()) * m_xboxController.getLeftX()) * kMaxSpeed)
                            .withRotationalRate(-m_xboxController.getRightX() * kMaxAngularRate)
                            .withCenterOfRotation(RotationCenter)
                    );
                } else {
                    drivetrain.setControl(
                        kRobotCentricControl
                            .withVelocityX(-(Math.abs(m_xboxController.getLeftY()) * m_xboxController.getLeftY()) * kMaxSpeed)
                            .withVelocityY(-(Math.abs(m_xboxController.getLeftX()) * m_xboxController.getLeftX()) * kMaxSpeed)
                            .withRotationalRate(-m_xboxController.getRightX() * kMaxAngularRate)
                            .withCenterOfRotation(RotationCenter)
                    );
                }
                
            }
        });
    }

    public Command driveToTargetCommand(CommandXboxController m_xboxController) {
        return run(() -> {
            double fieldCentricX = (Math.abs(m_xboxController.getLeftY()) * m_xboxController.getLeftY());
            double fieldCentricY = (Math.abs(m_xboxController.getLeftX()) * m_xboxController.getLeftX());

            if (Robot.isRedAlliance()) {
                fieldCentricX = fieldCentricX * -1;
                fieldCentricY = fieldCentricY * -1;
            }

            aprilTagTargetTracker.recalculateDriveFeedback(getCurrentPose(), fieldCentricX, fieldCentricY);

            if (aprilTagTargetTracker.hasVisibleTarget()) {
                lostTargetCount = 0;
                lockedOnce = true;
            } else {
                lostTargetCount++;
            }

            if (aprilTagTargetTracker.hasVisibleTarget() || ((lostTargetCount < 5) && lockedOnce == true)) {
                // note to self since this is confusing... Y becomes X and X becomes Y and that change happens earlier for driving at target
                drivetrain.setControl(
                        driveAtTarget.withVelocityX(aprilTagTargetTracker.getCalculatedX() * kMaxSpeed)
                                .withVelocityY(  aprilTagTargetTracker.getCalculatedY() *kMaxSpeed)
                                .withTargetDirection(aprilTagTargetTracker.getRotationTarget()));
            } else {
                drivetrain.setControl(
                        kFieldCentricControl
                                .withVelocityX(-(Math.abs(m_xboxController.getLeftY()) * m_xboxController.getLeftY())
                                        * kMaxSpeed)
                                .withVelocityY(-(Math.abs(m_xboxController.getLeftX()) * m_xboxController.getLeftX())
                                        * kMaxSpeed)
                                .withRotationalRate(-m_xboxController.getRightX() * kMaxAngularRate));
            }
        });
    }

    public Command pathfindToPoseCommand(Pose2d targetPose, double endVelocity) {
        return AutoBuilder.pathfindToPose(
            targetPose, 
            kPathfinderConstraints, 
            endVelocity
        );
    }
}