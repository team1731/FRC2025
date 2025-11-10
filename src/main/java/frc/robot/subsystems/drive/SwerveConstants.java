package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.util.Units;

import com.ctre.phoenix6.swerve.SwerveRequest;

import frc.robot.subsystems.drive.generated.TunerConstants;
import frc.robot.subsystems.vision.VisionConstants;

public class SwerveConstants {
    public static final double kMaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // Max robot speed
    public static final double kMaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    public static final double kDeadband = 0.05; // 5% joystick deadband

    public static final boolean kUseVSLAM = true;
    public static final boolean kUseAprilTags = true;

    public static final PIDConstants kPPConstants = new PIDConstants(10d, 0d, 0d); // PID constants for PathPlanner path following

    public static final PathConstraints kPathfinderConstraints = new PathConstraints(
        2.0, 2.0, 
        Units.degreesToRadians(360), Units.degreesToRadians(540)
    );

    /* Setting up bindings for necessary control of the swerve drive platform */
    public static final SwerveRequest.FieldCentric kFieldCentricControl = new SwerveRequest.FieldCentric()
        .withDeadband(kMaxSpeed * kDeadband) // Add a 5% deadband
        .withRotationalDeadband(kMaxAngularRate * kDeadband) // Add a 5% deadband
        .withDriveRequestType(DriveRequestType.Velocity); // Use open-loop control for drive motors

    public static final SwerveRequest.RobotCentric kRobotCentricControl = new SwerveRequest.RobotCentric()
        .withDeadband(kMaxSpeed * kDeadband) // Add a 5% deadband
        .withRotationalDeadband(kMaxAngularRate * kDeadband) // Add a 5% deadband
        .withDriveRequestType(DriveRequestType.Velocity);

    private static final double kDriveToTargetMaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private static final double kDriveToTargetDeadband = 0.01; // Add a 1% deadband
    public static final SwerveRequest.FieldCentricFacingAngle driveAtTarget = new SwerveRequest.FieldCentricFacingAngle()
        .withRotationalDeadband(VisionConstants.MAX_ANGULAR_SPEED * kDriveToTargetDeadband) // Add a 1% deadband
		.withDriveRequestType(DriveRequestType.OpenLoopVoltage)
        .withDeadband((kDriveToTargetMaxSpeed * kDeadband));
    
    public static final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    public static final SwerveRequest.ApplyRobotSpeeds autoRequest = new SwerveRequest.ApplyRobotSpeeds();
}