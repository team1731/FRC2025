package frc.robot;

import java.util.Map;

// import com.ctre.phoenix6.configs.FeedbackConfigs;
// import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.util.Gains;
import frc.robot.util.log.LogWriter.Log;
import frc.robot.util.log.LogWriter.LogMode;

public final class Constants {
    public static final double stickDeadband = 0.1;
    public static final int kTICKS = 33024; // 16.125 * 2048;
    public static final String CANBUS_NAME = "canivore1";

    public static final class LogConstants {
        /*
         * To write to a log you must:
         * 1. Set loggingEnabled = true
         * 2. Set the desired logMode (CSV, DATA_LOG)
         * 2. Set the desired loggers below = true
         */
        public static final boolean loggingEnabled = false; // note: must also turn on applicable loggers below
        public static final boolean logNetworkTables = false; // only applicable when logMode = DATA_LOG
        public static final LogMode logMode = LogMode.CSV;

        // list of loggers and enabled status, note: must also enable logging above
        public static final Map<Log, Boolean> loggers = Map.of(
                Log.MESSAGE, false,
                Log.ARM_PATH_RECORDING, false,
                Log.POSE_ESTIMATIONS, false);

        // Arm path recording constants
        public final static double recordingPeriod = 0.01; // seconds
        public final static double recordingOffset = 0.005; // seconds
    }

    public static final class FieldConstants {
        // Note: Field dimensions and April Tag positions pulled from the 2023 Field and
        // Layout Marking document
        public static final double kFieldLength = Units.inchesToMeters(651.22);
        public static final double kFieldWidth = Units.inchesToMeters(315.1);

        public static class AprilTagPoseValues {
            public int id;
            public double x;
            public double y;
            public double z;
            public double yaw;

            public AprilTagPoseValues(int tagId, double xInches, double yInches, double zInches, double yawDegrees) {
                id = tagId;
                x = Units.inchesToMeters(xInches);
                y = Units.inchesToMeters(yInches);
                z = Units.inchesToMeters(zInches);
                yaw = Units.degreesToRadians(yawDegrees);
            }

            public AprilTag getAprilTag() {
                return new AprilTag(id, new Pose3d(x, y, z, new Rotation3d(0.0, 0.0, yaw)));
            }
        }

        public static final AprilTagPoseValues kAprilTagPose1 = new AprilTagPoseValues(1, 610.77, 42.19, 18.22, 180);
        public static final AprilTagPoseValues kAprilTagPose2 = new AprilTagPoseValues(2, 610.77, 108.19, 18.22, 180);
        public static final AprilTagPoseValues kAprilTagPose3 = new AprilTagPoseValues(3, 610.77, 174.19, 18.22, 180);
        public static final AprilTagPoseValues kAprilTagPose4 = new AprilTagPoseValues(4, 636.96, 265.74, 27.38, 180);
        public static final AprilTagPoseValues kAprilTagPose5 = new AprilTagPoseValues(5, 14.25, 265.74, 27.38, 0);
        public static final AprilTagPoseValues kAprilTagPose6 = new AprilTagPoseValues(6, 40.45, 174.19, 18.22, 0);
        public static final AprilTagPoseValues kAprilTagPose7 = new AprilTagPoseValues(7, 40.45, 108.19, 18.22, 0);
        public static final AprilTagPoseValues kAprilTagPose8 = new AprilTagPoseValues(8, 40.45, 42.19, 18.22, 0);
    }

    public static final class AutoConstants {
        public static final String kAutoDefault = "2_test"; // TODO replace this when we have a real default auto
        public static final String kAutoCodeKey = "Auto Selector";

        public static final double kMaxSpeedMetersPerSecond = 0.5; // disabled for testing = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 0.5; // disabled for testing = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

        public static final double kPXController = 3.0;
        public static final double kPYController = 3.0;
        public static final double kPThetaController = 2;

        // Constraint for the motion profilied robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static class Vision {
        public static final double kTargetConfidenceDelta = 0.5;

        public static final String kCameraNameFront = "Global_Shutter_Camera";
        public static final String kCameraNameBack = "ArducamUSB1";
        public static final double kMaxDistanceBetweenPoseEstimations = 1.0;

        // Cam mounted facing forward, half a meter forward of center, half a meter up from center.
        public static final Transform3d kRobotToCamFront = new Transform3d(new Translation3d(-0.336555, -0.054, 0.4572),
                new Rotation3d(0, -Units.degreesToRadians(12), Units.degreesToRadians(180)));
        public static final Transform3d kRobotToCamBack = new Transform3d(new Translation3d(-0.336555, 0, 0.4572),
                new Rotation3d(0, -Units.degreesToRadians(30.5), Units.degreesToRadians(180)));

        // The layout of the AprilTags on the field
        public static final AprilTagFieldLayout kTagLayout = AprilTagFields.kDefaultField.loadAprilTagLayoutField();

        // The standard deviations of our vision estimated poses, which affect correction rate
        // (Fake values. Experiment and determine estimation noise on an actual robot.)
        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(2, 2, 8);
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.05, 0.05, 1);
        public static final Matrix<N3, N1> kTrapStdDevs = VecBuilder.fill(0.1, 0.1, 0.001);
        public static final Matrix<N3, N1> kVSLAMStdDevs = VecBuilder.fill(0.00001, 0.00001, 0.00001);
    }
    public static final class OpConstants {
        // KEYBOARD CONSTANTS
        public static final int kPWM_LedString = 1; // PWM # for Addressable Led String
        public static final int kLedStringLength = 11; // Length of Addressable Led String; 33 individual / sets of 3
        public static final double kLedStringBlinkDelay = 0.1; // Delay in Seconds of Addressable Led String

        public enum LedOption {
            INIT, YELLOW, PURPLE, BLACK, WHITE, BLUE, RED, GREEN
        }
    }

    // public static final class VisionConstants {
    // // Ensure measurements are in METERS
    // public static final double kMaxDistanceBetweenPoseEstimations = 1.0;

    //     // Kalman Filter Configuration. These can be "tuned-to-taste" based on how much you trust your various sensors. 
    //     // Smaller numbers will cause the filter to "trust" the estimate from that particular component more than the 
    //     // others. This in turn means the particular component will have a stronger influence on the final pose estimate.
    //     public static final Matrix<N3, N1> kVisionMeasurementStdDevs = VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(180));

    // public static class CameraMountPoseValues {
    // public String id;
    // public double x;
    // public double y;
    // public double z;
    // public double yaw;
    // public double pitch;

    // // Note: this constructor assumes the camera is mounted parallel to the floor
    // public CameraMountPoseValues(String cameraId, double xInches, double yInches,
    // double zInches, double yawDegrees, double pitchDegrees) {
    // id = cameraId;
    // x = Units.inchesToMeters(xInches);
    // y = Units.inchesToMeters(yInches);
    // z = Units.inchesToMeters(zInches);
    // yaw = Units.degreesToRadians(yawDegrees);
    // pitch = Units.degreesToRadians(pitchDegrees);
    // }

    // public Transform3d getPoseTransform() {
    // return new Transform3d(new Translation3d(x, y, z), new
    // Rotation3d(0.0,pitch,yaw));
    // }
    // }

    // public static final String kCameraMount1Id = "leftcamera"; //camera on the
    // left looking back
    // public static final String kCameraMount2Id = "Global_Shutter_Camera"; //
    // camera on the right looking back
    // public static final String kCameraMount3Id = "USB_Camera2"; // camera on the
    // arm
    // public static final CameraMountPoseValues kCameraMount1Pose = new
    // CameraMountPoseValues(kCameraMount1Id, -5.3,10.253, 17.0, 135.0,0.0);
    // public static final CameraMountPoseValues kCameraMount2Pose = new
    // CameraMountPoseValues(kCameraMount2Id, -5.3, -10.253, 17.0, 225.0,0.0);
    // public static final CameraMountPoseValues kCameraMount3Pose = new
    // CameraMountPoseValues(kCameraMount3Id, -11, -5.0, 37, 0.0,-80.0);

    // // #region TurnPID
    // public static final double kTurnP = 0.05;
    // public static final double kTurnI = 0;
    // public static final double kTurnD = 0.00;
    // // public static final double kMaxTurnVelocity = 360;
    // // public static final double kMaxTurnAcceleration = 360;
    // public static final double kTurnToleranceDeg = 5;
    // public static final double kTurnRateToleranceDegPerS = 10; // degrees per
    // second
    // // #endregion
    // }

}
