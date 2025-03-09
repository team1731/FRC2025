package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

public class VisionConstants {
    /*
     * PHOTONVISION CONSTANTS
     */
    public static final String camera1Name = "ElevSide";
    public static final String camera2Name = "BattSide";

    // AprilTag drive targeting constants
    public static final double VISION_FORWARD_kP = 1;
    public static final double VISION_STRAFE_kP = 0.1;
    public static final double VISION_ROTATE_kP = 1;
    public static final double MAX_ANGULAR_SPEED = RotationsPerSecond.of(0.1).in(RadiansPerSecond);
    public static final double MAX_LINEAR_SPEED = 0.5;

    public static final double targetConfidenceDelta = 0.5;

    // The standard deviations of our vision estimated poses, which affect correction rate
    // (Fake values. Experiment and determine estimation noise on an actual robot.)
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(2, 2, 8);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.05, 0.05, 1);
    public static final Matrix<N3, N1> kTrapStdDevs = VecBuilder.fill(0.1, 0.1, 0.001);
    public static final Matrix<N3, N1> kVSLAMStdDevs = VecBuilder.fill(0.00001, 0.00001, 0.00001);
}