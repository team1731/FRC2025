package frc.robot.subsystems.drive;

import java.util.function.Supplier;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.subsystems.ToggleableSubsystem;
import frc.robot.subsystems.vision.AprilTagSubsystem;
import frc.robot.subsystems.vision.VSLAMSubsystem;
import static edu.wpi.first.units.Units.*;
import frc.robot.autos.AutoFactory;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;


/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements
 * subsystem
 * so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements ToggleableSubsystem {
    private boolean enabled;
    private final SwerveRequest.ApplyRobotSpeeds autoRequest = new SwerveRequest.ApplyRobotSpeeds();

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;
    private boolean m_hasConfiguredInitialPosition = false;

    /* AprilTag vision */
    private boolean useAprilTags = true;
    private AprilTagSubsystem aprilTagSubsystem;

    /* VSLAM Updates */
    private boolean useVSLAM = true;
    private VSLAMSubsystem vslamSubsystem;
    private DrivetrainVisionCallback visionCallback = (Pose2d pose, double timestamp, Matrix<N3,N1> visionMeasurementStdDevs) -> {
        this.addVisionMeasurement(pose, timestamp, visionMeasurementStdDevs);  // comment this out to disable vslam
    };
    private DrivetrainSetPoseCallback setPoseCallback = () -> {
          this.configureInitialPosition();  // comment this out to disable vslam
    };

    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    /*
     * SysId routine for characterizing translation. This is used to find PID gains
     * for the drive motors.
     */
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
            new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
                    Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
            null,        // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    output -> setControl(m_translationCharacterization.withVolts(output)),
                    null,
                    this));

    /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
            new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
                    Volts.of(7), // Use dynamic voltage of 7 V
            null,        // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdSteer_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    volts -> setControl(m_steerCharacterization.withVolts(volts)),
                    null,
                    this));

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
     */
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    /* This is in radians per second², but SysId only supports "volts per second" */
                    Volts.of(Math.PI / 6).per(Second),
                    /* This is in radians per second, but SysId only supports "volts" */
                    Volts.of(Math.PI),
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdRotation_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    output -> {
                        /* output is actually radians per second, but SysId only supports "volts" */
                        setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                        /* also log the requested output for SysId */
                        SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
                    },
                    null,
                    this));

    /* The SysId routine to test */
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

    private void setEnabled(boolean enabled) {
        this.enabled = enabled;
    }
    

    @Override
    public boolean isEnabled() {
        return enabled;
    }

    public CommandSwerveDrivetrain(boolean enabled, SwerveDrivetrainConstants driveTrainConstants,
            double OdometryUpdateFrequency, SwerveModuleConstants<?, ?, ?>... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        setEnabled(enabled);
        if(!enabled) return;
        if(useAprilTags) aprilTagSubsystem = new AprilTagSubsystem(true);
        if(useVSLAM) {
            vslamSubsystem = new VSLAMSubsystem(visionCallback, setPoseCallback);
            vslamSubsystem.configure();
        }
    }

    public CommandSwerveDrivetrain(boolean enabled, SwerveDrivetrainConstants driveTrainConstants,
            SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        setEnabled(enabled);
        if(!enabled) return;
        if(useAprilTags) aprilTagSubsystem = new AprilTagSubsystem(true);
        if(useVSLAM) {
            vslamSubsystem = new VSLAMSubsystem(visionCallback, setPoseCallback);
            vslamSubsystem.configure();
        }
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        if(!enabled) return new Command() {};
        return run(() -> this.setControl(requestSupplier.get()));
    }

    public ChassisSpeeds getCurrentRobotChassisSpeeds() {
        if (!enabled || Robot.isSimulation())
            return new ChassisSpeeds();
        return getKinematics().toChassisSpeeds(getState().ModuleStates);
    }
    
    public AprilTagSubsystem getAprilTagSubsystem() {
        return useAprilTags? aprilTagSubsystem : null;
    }
    public VSLAMSubsystem getVSLAMSubsytem() {
        return useVSLAM? vslamSubsystem : null;
    }

    /*
     * VSLAM DRIVE TO POSE
     */

    public Command driveToPose(Pose2d targetPose) {
        return AutoBuilder.pathfindToPose(
            targetPose, 
            new PathConstraints(
                2.0, 2.0, 
                Units.degreesToRadians(360), Units.degreesToRadians(540)
            ), 
            0
        );
    }

    /*
     * PERIODIC HANDLING
     */

    public void periodic() {
        if(useVSLAM) {
            vslamSubsystem.cleanUpSubroutineMessages(); 
            vslamSubsystem.processHeartbeat();
        }

        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is disabled.
         * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
         */
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                        allianceColor == Alliance.Red
                                ? kRedAlliancePerspectiveRotation
                                : kBlueAlliancePerspectiveRotation);
                m_hasAppliedOperatorPerspective = true;
            });
        }
    }

    public double getXVelocity() {
        var chassisSpeeds = getCurrentRobotChassisSpeeds();
        return chassisSpeeds.vxMetersPerSecond;
    }

    public double getYVelocity() {
        var chassisSpeeds = getCurrentRobotChassisSpeeds();
        return chassisSpeeds.vyMetersPerSecond;

    }

    /**
     * Log the torque current and velocity
     */
    public void logCurrentAndVelocity() {

        // Iterate through each module.
        for (int i = 0; i < 4; ++i) {
            // Get the Configurator for the current drive motor.
            SmartDashboard.putNumber("Module " + i + "Torque Current",
                    getModules()[i].getDriveMotor().getTorqueCurrent().getValueAsDouble());
            SmartDashboard.putNumber("Module " + i + "Velocity",
                    getModules()[i].getDriveMotor().getVelocity().getValueAsDouble());
        }
    }

    /** Log various drivetrain values to the dashboard. */
    public void log() {
        // Nothing defined yet
    }

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }
    
    /*
     * This method will get called in two instances:
     * 1. After the VSLAM connects successfully
     * 2. When the alliance changes
     */
    public void configureInitialPosition() {
        // If using VSLAM, don't configure the position until the VSLAM is connected
        if((useVSLAM && vslamSubsystem == null) || (vslamSubsystem != null && !vslamSubsystem.isConnected())) return;

        System.out.println("CommandSwerveDrivetrain: configuring a new position");

	    Pose2d startingConfiguration = Robot.isRedAlliance()? 
            new Pose2d(10.38, 3.01, new Rotation2d(0)) : 
            new Pose2d(7.168, 5.006, new Rotation2d(Math.toRadians(180)));
        resetPose(startingConfiguration);

        Rotation2d operatorPerspective = Robot.isRedAlliance()? 
                new Rotation2d(Math.toRadians(180)) : 
                new Rotation2d(Math.toRadians(0));
        setOperatorPerspectiveForward(operatorPerspective);
    }

    public void configureAutoBindings() {
        try {
            var config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                ()->this.getState().Pose,
                this::resetPose, 
                this::getCurrentRobotChassisSpeeds, 
                // Consumer of ChassisSpeeds to drive the robot
                (speeds, feedsforwards)->this.setControl(autoRequest.withSpeeds(speeds)),
                new PPHolonomicDriveController(
                    new PIDConstants(10, 0, 0),
                    new PIDConstants(10, 0, 0)),
                config,
                () -> AutoFactory.isFlipRedBlue(),
                this);
        } catch(Exception e) {
            System.out.println("CommandSwerveDrivetrain error - failed to configure auto bindings");
        }
    }

    public void zeroHeading() {
        vslamSubsystem.zeroHeading();
    }

    public Pose2d getCurrentPose() {
        return this.getState().Pose;
    }

    // Zero the absolute 3D position of the robot
    @Override
    public void resetPose(Pose2d position) {
        System.out.println("Adjusting the position of the robot x=" + position.getX() +  " y = " + position.getY() + " rotation = " + position.getRotation().getDegrees());
        super.resetPose(position);
        if(useVSLAM) {
            vslamSubsystem.calculateNewOffset(position);
        }
    }
}
