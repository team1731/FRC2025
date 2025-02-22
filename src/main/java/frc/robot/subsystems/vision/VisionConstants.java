package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

public class VisionConstants {
    /*
     * PHOTONVISION CONSTANTS
     */
    public static final double targetConfidenceDelta = 0.5;

    public static final String camera1Name = "REPLACE";
    public static final String camera2Name = "REPLACE";
    public static final double maxDistanceBetweenPoseEstimations = 1.0;
    public static final double cameraHeight = 7.25; // inches
    public static final double cameraPitch = 5.0; // degrees, measured with a protractor, or in CAD

    public static final Transform2d robotToCamera2d = new Transform2d(Units.inchesToMeters(-8.5), Units.inchesToMeters(11.5), new Rotation2d(Math.toDegrees(22)));
    public static final Pose2d leftReefScorePose = new Pose2d(Units.inchesToMeters(-12.5), Units.inchesToMeters(6.25), new Rotation2d());
    public static final Pose2d rightReefScorePose =  new Pose2d(Units.inchesToMeters(-12.5), Units.inchesToMeters(-6.25), new Rotation2d());

    // AprilTag drive targeting constants
    public static final double VISION_FORWARD_kP = 1;
    public static final double VISION_STRAFE_kP = 0.1;
    public static final double VISION_ROTATE_kP = 1;
    public static final double MAX_ANGULAR_SPEED = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
    public static final double MAX_LINEAR_SPEED = 0.5;

    // The standard deviations of our vision estimated poses, which affect correction rate
    // (Fake values. Experiment and determine estimation noise on an actual robot.)
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(2, 2, 8);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.05, 0.05, 1);
    public static final Matrix<N3, N1> kTrapStdDevs = VecBuilder.fill(0.1, 0.1, 0.001);
    public static final Matrix<N3, N1> kVSLAMStdDevs = VecBuilder.fill(0.00001, 0.00001, 0.00001);
}