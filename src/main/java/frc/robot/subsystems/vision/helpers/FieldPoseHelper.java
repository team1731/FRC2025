package frc.robot.subsystems.vision.helpers;

import java.util.Arrays;
import java.util.List;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FieldPoseHelper {
    // Reef post relative to the April Tag
    public static enum ReefPost {
        LEFT_POST,
        RIGHT_POST
    }

    // Measurement constants
    public static final double inchesLateralToLeftPost = -6.25;
    public static final double inchesLateralToRightPost = 6.25;
    public static final double inchesToCenterOfRobot = 12.5 + (3/4) + 2.5 + (1/8);
    public static final double reefAprilTagHeight = 11.75;
    
    // The layout of the AprilTags on the field
    public static final AprilTagFieldLayout kTagLayout = AprilTagFields.kDefaultField.loadAprilTagLayoutField(); 
    // Auto lineup targets
    // LEFT/CENTER/RIGHT are from the drive team's perspective
    public static int redAllianceAutoLineupTargetId = 10;
    public static int blueAllianceAutoLineupTargetId = 21;
    public static double autoLineupLeftPositionThreshold = -15;
    public static double autoLineupRightPositionThreshold = 15;
    public static double autoLineupTolerance = 0.2;
    public static double autoLineupLeftTargetYaw =-25.3;  //ok  Barge battside    
    public static double autoLineupCenterTargetYaw = 0.28; // need to uise battside
    public static double autoLineupRightTargetYaw = 27.76; // ok Processor side elevside  

    // List of April Tag poses on the reef
    /*
    public static Pose3d[] reefAprilTagPoses = {
        // Red Alliance Coral Reef
        kTagLayout.getTagPose(6).get(),
        kTagLayout.getTagPose(7).get(),
        kTagLayout.getTagPose(8).get(),
        kTagLayout.getTagPose(9).get(),
        kTagLayout.getTagPose(10).get(),
        kTagLayout.getTagPose(11).get(),
        // Blue Alliance Coral Reef
        kTagLayout.getTagPose(17).get(),
        kTagLayout.getTagPose(18).get(),
        kTagLayout.getTagPose(19).get(),
        kTagLayout.getTagPose(20).get(),
        kTagLayout.getTagPose(21).get(),
        kTagLayout.getTagPose(22).get(),
    };
    */
    public static final int[] reefAprilTagIds = { 6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22 };
    public static final int[][] reefAprilTagRotations = { 
        // Tag ID, Tag Rotation
        { 6, 300 },
        { 7, 0 },
        { 8, 60 },
        { 9, 120 }, 
        { 10, 180 },
        { 11, 240 },
        { 17, 240 },
        { 18, 180 }, 
        { 19, 120 }, 
        { 20, 60 },
        { 21, 0 }, 
        { 22, 300 }
    };    

    /*
     * List of line up poses for reef scoring
     * There will be twice as many as there are reef april tags --> one left post lineup pose, and one right post lineup pose
     */
    public static List<Pose2d> coralReefLineupPoses;

    public static boolean isReefTarget(int targetId) {
        return Arrays.stream(reefAprilTagIds).anyMatch(i -> i == targetId);
    }

    public static Rotation2d getReefTargetRotation(int targetId) {
        double targetRotation = 0.0;
        for(int[] rotation : reefAprilTagRotations) {
            if(targetId == rotation[0]) {
                targetRotation = (double)rotation[1];
            }
        }

        return new Rotation2d(Units.degreesToRadians(targetRotation));
    }

    public static Rotation2d getReefTargetLineupRotation(int targetId) {
        Rotation2d targetRotation = null;
        Rotation2d lineupRotation = null;
        for(int[] rotation : reefAprilTagRotations) {
            if(targetId == rotation[0]) {
                targetRotation = new Rotation2d(Units.degreesToRadians((double)rotation[1]));
                lineupRotation = getRotation180(targetRotation);
                break;
            }
        }

        if(targetRotation != null) SmartDashboard.putNumber("tag rotation from list", targetRotation.getDegrees());
        if(lineupRotation != null) SmartDashboard.putNumber("tag rotation after 180", lineupRotation.getDegrees());

        return lineupRotation;
    }

    /* Currently disabled b/c field layout loading is disables (see above)
    public static Pose2d getClosestReefLineupPose(Pose2d currentPose) {
        return currentPose.nearest(getCoralReefLineupPoses());
    }
    
    public static List<Pose2d> getCoralReefLineupPoses() {
        if(coralReefLineupPoses == null) {
            coralReefLineupPoses = new ArrayList<Pose2d>();
            // cycle through the coral reef april tags and add a lineup pose for each of its left and right scoring posts
            for(Pose3d aprilTagPose : reefAprilTagPoses) {
                coralReefLineupPoses.add(getLineupPose(aprilTagPose, ReefPost.LEFT_POST));
                coralReefLineupPoses.add(getLineupPose(aprilTagPose, ReefPost.RIGHT_POST));
            }
        }
        return coralReefLineupPoses;
    }
    */

    public static Pose2d getLineupPose(Pose3d aprilTagPose, ReefPost post) {
        // the distance laterally to the left or right post
        double shiftToPost = (post == ReefPost.LEFT_POST)? inchesLateralToLeftPost : inchesLateralToRightPost;
        // the angle to face the post, which is 180 degrees from the april tag
        Rotation2d rotation = getRotation180(aprilTagPose.getRotation());
        // calculate the transform using the lateral shift, the distance to the center of the bot, and the rotation
        Transform2d transform = new Transform2d(Units.inchesToMeters(shiftToPost), Units.inchesToMeters(inchesToCenterOfRobot), rotation);
        // apply the transformation to the april tag pose to get the correct lineup pose
        return aprilTagPose.toPose2d().transformBy(transform);
    }

    private static Rotation2d getRotation180(Rotation3d initialRotation) {
        Rotation3d rotation180 = new Rotation3d(new Rotation2d(Units.degreesToRadians(180)));
        return initialRotation.plus(rotation180).toRotation2d();
    }

    private static Rotation2d getRotation180(Rotation2d initialRotation) {
        Rotation2d rotation180 = new Rotation2d(Units.degreesToRadians(180));
        return initialRotation.plus(rotation180);
    }
}