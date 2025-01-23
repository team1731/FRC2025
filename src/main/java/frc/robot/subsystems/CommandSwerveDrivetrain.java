package frc.robot.subsystems;

import java.util.EnumSet;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.FloatArraySubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import static frc.robot.Constants.Vision.*;

// (SCH) FIXME: These imports don't seem to exist anymore in 2025.
// You'll need to check the Pathplanner docs for what happened to these classes
import com.pathplanner.lib.config.PIDConstants;
//import com.pathplanner.lib.util.PIDConstants;
//import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
//import com.pathplanner.lib.util.ReplanningConfig;


import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.Vision;
import frc.robot.generated.TunerConstants;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.subsystems.ToggleableSubsystem;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import com.ctre.phoenix6.swerve.jni.SwerveJNI.ControlParams;

class FlipRedBlueSupplier implements BooleanSupplier {
    @Override
    public boolean getAsBoolean() {
        return RobotContainer.isFlipRedBlue();
    }
}

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem
 * so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements ToggleableSubsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    private boolean enabled;
    private final SwerveRequest.ApplyRobotSpeeds autoRequest = new SwerveRequest.ApplyRobotSpeeds();

         /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    /* VSLAM Updates */
    boolean useVSLAM = true;
    private final Field2d vslamfield = new Field2d();
    int connListenerHandle;
    int positionListenerHandle;
    int topicListenerHandle;

    private IntegerSubscriber questMiso;
    private IntegerPublisher questMosi;
    private IntegerSubscriber questFrameCount;
    private DoubleSubscriber questTimestamp;
    private FloatArraySubscriber questPosition;
    private FloatArraySubscriber questQuaternion;
    private FloatArraySubscriber questEulerAngles;
    private DoubleSubscriber questBattery;

    private float yaw_offset = 0.0f;
    private Pose2d startingOffset = new Pose2d();

    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> setControl(m_translationCharacterization.withVolts(output)),
            null,
            this
        )
    );

    /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(7), // Use dynamic voltage of 7 V
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            volts -> setControl(m_steerCharacterization.withVolts(volts)),
            null,
            this
        )
    );

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
            state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> {
                /* output is actually radians per second, but SysId only supports "volts" */
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                /* also log the requested output for SysId */
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
            },
            null,
            this
        )
    );

    /* The SysId routine to test */
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;
    
    private void setEnabled(boolean enabled) {
        this.enabled = enabled;
    }

    @Override
    public boolean isEnabled() {
        return enabled;
    }

    public CommandSwerveDrivetrain(boolean enabled, SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency, SwerveModuleConstants<?, ?, ?>... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        setEnabled(enabled);
        if(!enabled) return;
        //configurePathPlanner(true);
        configureVSLAM();
    }
    
    public CommandSwerveDrivetrain(boolean enabled, SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        setEnabled(enabled);
        if(!enabled) return;
        //configurePathPlanner(true);
        configureVSLAM();
    }

        public void configureVSLAM() {

        NetworkTableInstance inst = NetworkTableInstance.getDefault();

        // add a connection listener; the first parameter will cause the
        // callback to be called immediately for any current connections
        connListenerHandle = inst.addConnectionListener(true, event -> {
            if (event.is(NetworkTableEvent.Kind.kConnected)) {
                System.out.println("Connected to " + event.connInfo.remote_id);
            } else if (event.is(NetworkTableEvent.Kind.kDisconnected)) {
                System.out.println("Disconnected from " + event.connInfo.remote_id);
            }
        });

        // get the subtable called "questnav"
        NetworkTable datatable = inst.getTable("questnav");
        questMiso = datatable.getIntegerTopic("miso").subscribe(0);
        questMosi = datatable.getIntegerTopic("mosi").publish();
        questFrameCount = datatable.getIntegerTopic("frameCount").subscribe(0);
        questTimestamp = datatable.getDoubleTopic("timestamp").subscribe(0.0f);
        questPosition = datatable.getFloatArrayTopic("position")
                .subscribe(new float[] { 0.0f, 0.0f, 0.0f });
        questQuaternion = datatable.getFloatArrayTopic("quaternion")
                .subscribe(new float[] { 0.0f, 0.0f, 0.0f, 0.0f });
        questEulerAngles = datatable.getFloatArrayTopic("eulerAngles")
                .subscribe(new float[] { 0.0f, 0.0f, 0.0f });
        questBattery = datatable.getDoubleTopic("batteryLevel").subscribe(0.0f);
       
        
        ShuffleboardTab tab = Shuffleboard.getTab("test");
  
        tab.add(vslamfield);

       
        System.out.println("addind listener******************************************8");
        // add a listener to only value changes on the Y subscriber
        positionListenerHandle = inst.addListener(
                questPosition,
                EnumSet.of(NetworkTableEvent.Kind.kValueAll),
                event -> {

                    var timestampedPosition = questPosition.getAtomic();
                    float[] oculusPosition = timestampedPosition.value;
                    double timestamp = timestampedPosition.timestamp;
                    timestamp = timestamp/1000000;
                    Translation2d oculousRawPosition = new Translation2d(-oculusPosition[2], oculusPosition[0]);
                    Translation2d  oculousPositionCompensated = oculousRawPosition.plus(new Translation2d(0.3333* Math.cos(Math.toRadians(getOculusYaw())), 0.333*Math.sin(Math.toRadians(getOculusYaw())))); // TODO GET Numbers since robot is not in the center of the robot
                    oculousPositionCompensated = oculousPositionCompensated.plus(startingOffset.getTranslation());  // translate by the starting position

                    Rotation2d oculousRawRotation = Rotation2d.fromDegrees(getOculusYaw()).plus(Rotation2d.fromDegrees(0));  // since camera is on back of robot
                    Rotation2d  oculousCompensatedRotation = oculousRawRotation.plus(startingOffset.getRotation());
                    
                    Pose2d estPose = new Pose2d(oculousPositionCompensated, oculousCompensatedRotation);
                    
                    //System.out.println("addind a vslam");
                    vslamfield.getObject("MyRobotVSLAM").setPose(estPose);
                    SmartDashboard.putString("VSLAM pose", String.format("(%.2f, %.2f) %.2f %.2f %.2f",
                            estPose.getTranslation().getX(),
                            estPose.getTranslation().getY(),
                            estPose.getRotation().getDegrees(),
                            timestamp,
                            Timer.getFPGATimestamp()));
                    if (useVSLAM) {
                        this.addVisionMeasurement(estPose,
                               timestamp, kVSLAMStdDevs);
                    } 
                });

        // add a listener to see when new topics are published within datatable
        // the string array is an array of topic name prefixes.
        topicListenerHandle = inst.addListener(
                new String[] { datatable.getPath() + "/" },
                EnumSet.of(NetworkTableEvent.Kind.kTopic),
                event -> {
                    if (event.is(NetworkTableEvent.Kind.kPublish)) {
                        // topicInfo.name is the full topic name, e.g. "/datatable/X"
                        System.out.println("newly published " + event.topicInfo.name);
                    }
                });
            }

     public void configurePathPlanner(boolean redBlueFlipping) {
         if(!enabled) return;
         double driveBaseRadius = 0;
  
        for (var moduleLocation : getModuleLocations()) {
             driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
         }

    //     System.out.println("Configuring AutoBuilder!");
    // //FIXME:Broken Pathplanner libaries, disabled to build code
    //     // (SCH) FIXME: This is related to the Pathbuilder imports at the top of the file
    //      AutoBuilder.configureHolonomic(
    //          ()->this.getState().Pose, // Supplier of current robot pose
    //          this::seedFieldRelative,  // Consumer for seeding pose against auto
    //          this::getCurrentRobotChassisSpeeds,
    //          (speeds)->this.setControl(autoRequest.withSpeeds(speeds)), // Consumer of ChassisSpeeds to drive the robot
    //          // (SCH) FIXME: This is related to the Pathbuilder imports at the top of the file
    //          new HolonomicPathFollowerConfig(new PIDConstants(10, 0, 0),
    //                                          new PIDConstants(10, 0, 0),
    //                                         TunerConstants.kSpeedAt12VoltsMps,
    //                                          driveBaseRadius,
    //                                         // (SCH) FIXME: This is related to the Pathbuilder imports at the top of the file
    //                                         new ReplanningConfig()),
    //          new FlipRedBlueSupplier(), // ()->false, // Change this if the path needs to be flipped on red vs blue
    //          this); // Subsystem for requirements
    }

            


    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        if(!enabled) return new Command(){};
        return run(() -> this.setControl(requestSupplier.get()));
    }

    public Command getAutoPath(String pathName) {
        if(!enabled) return new Command(){};
        return new PathPlannerAuto(pathName);
    }

    public ChassisSpeeds getCurrentRobotChassisSpeeds() {
        if(!enabled || Robot.isSimulation()) return new ChassisSpeeds();
        return getKinematics().toChassisSpeeds(getState().ModuleStates);
       // return getKinematics();
    }

    public void periodic() {
        vslamfield.setRobotPose(this.getState().Pose);
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
                        : kBlueAlliancePerspectiveRotation
                );
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
Log the torque current and velocity
 */
    public void logCurrentAndVelocity() {
       

        //Iterate through each module.
        for (int i = 0; i < 4; ++i) {
            //Get the Configurator for the current drive motor.
            SmartDashboard.putNumber("Module " + i + "Torque Current" ,getModules()[i].getDriveMotor().getTorqueCurrent().getValueAsDouble());
            SmartDashboard.putNumber("Module " + i + "Velocity" ,getModules()[i].getDriveMotor().getVelocity().getValueAsDouble());
        }
    }

        /** Log various drivetrain values to the dashboard. */
    public void log() {
        // String table = "Drive/";
        // Pose2d pose = this.getState().Pose;
        // SmartDashboard.putNumber(table + "X", pose.getX());
        // SmartDashboard.putNumber(table + "Y", pose.getY());
        // SmartDashboard.putNumber(table + "Heading", pose.getRotation().getDegrees());
        // ChassisSpeeds chassisSpeeds = getCurrentRobotChassisSpeeds();
        // SmartDashboard.putNumber(table + "VX", chassisSpeeds.vxMetersPerSecond);
        // SmartDashboard.putNumber(table + "VY", chassisSpeeds.vyMetersPerSecond);
        // SmartDashboard.putNumber(
        //         table + "Omega Degrees", Math.toDegrees(chassisSpeeds.omegaRadiansPerSecond));
        // SmartDashboard.putNumber(table + "Target VX", targetChassisSpeeds.vxMetersPerSecond);
        // SmartDashboard.putNumber(table + "Target VY", targetChassisSpeeds.vyMetersPerSecond);
        // SmartDashboard.putNumber(
        //         table + "Target Omega Degrees", Math.toDegrees(targetChassisSpeeds.omegaRadiansPerSecond));

        // for (SwerveModule module : swerveMods) {
        //     module.log();
        // }
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

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    public void zeroHeading() {
        float[] eulerAngles = questEulerAngles.get();
        yaw_offset = eulerAngles[1];
       // angleSetpoint = 0.0;
      }
    
      // Zero the absolute 3D position of the robot (similar to long-pressing the quest logo)
      @Override
      public void resetPose(Pose2d position) {
        System.out.println("Adjusting the position of the robot");
       super.resetPose(position);
       Translation2d cameraoffset = position.getTranslation().minus(new Translation2d(.33333,0));
        startingOffset = new Pose2d(cameraoffset,position.getRotation());
        if (questMiso.get() != 99) {
          questMosi.set(1);
        }
      }
    
      // Clean up oculus subroutine messages after processing on the headset
      public void cleanUpOculusMessages() {
        if (questMiso.get() == 99) {
          questMosi.set(0);
        }
      }
    
      // Return the robot heading in degrees, between -180 and 180 degrees
      public double getHeading() {
        return Rotation2d.fromDegrees(getOculusYaw()).getDegrees();
      }
    
      // Get the rotation rate of the robot
      public double getTurnRate() {
        return getOculusYaw() ; //* (DriveConstants.kGyroReversed ? -1.0 : 1.0);
      }
    
      // Get the yaw Euler angle of the headset
      private float getOculusYaw() {
        float[] eulerAngles = questEulerAngles.get();
        var ret = eulerAngles[1] - yaw_offset;
        ret %= 360;
        if (ret < 0) {
          ret += 360;
        }
        return ret*-1;
      }
    
      private Translation2d getOculusPosition() {
        float[] oculusPosition = questPosition.get();
        return new Translation2d(oculusPosition[2], -oculusPosition[0]);
      }
    
      private Pose2d getOculusPose() {
        var oculousPositionCompensated = getOculusPosition().minus(new Translation2d(0, 0.1651)); // 6.5
        return new Pose2d(oculousPositionCompensated, Rotation2d.fromDegrees(getOculusYaw()));
      }    
}

