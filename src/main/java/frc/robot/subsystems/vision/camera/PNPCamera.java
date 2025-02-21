package frc.robot.subsystems.vision.camera;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.vision.VisionConstants;


public class PNPCamera implements Camera {
    private String name;
    private Transform3d location;
    private PhotonCamera photonCamera;
    private PhotonPoseEstimator poseEstimator;
    private Pose2d estimatedPose;
    private double lastUpdateTS;

    public PNPCamera(String cameraName, Transform3d cameraLocation) {
        name = cameraName;
        location = cameraLocation;
    }

    public void initialize() {
        photonCamera = new PhotonCamera(name);
        AprilTagFieldLayout fieldLayout = AprilTagFields.kDefaultField.loadAprilTagLayoutField();
        poseEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, location);
        poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        System.out.println("Camera: Adding camera " + name + "!!!!!!! ");
    }

    public String getName() {
        return name;
    }

    public PhotonCamera getCamera() {
        return photonCamera;
    }

    public PhotonPoseEstimator getPoseEstimator() {
        return poseEstimator;
    }

    public double getLastUpdatedTS() {
        return lastUpdateTS;
    }

    public void setLastUpdatedTS(double timestamp) {
        lastUpdateTS = timestamp;
    }

    public void setEstimatedPose(Pose2d pose) {
        estimatedPose = pose;
    }

    public Pose2d getEstimatedPose() {
        return estimatedPose;
    }

    public boolean isStale() {
        return ((Timer.getFPGATimestamp() - lastUpdateTS) > VisionConstants.targetConfidenceDelta);
    }

    public PhotonPipelineResult getLatestResult() {
        return photonCamera.getLatestResult();
    }

    public double getDistanceToTarget(Pose2d target) {
        Pose2d adjustedRobotPose = estimatedPose;
        double distance = PhotonUtils.getDistanceToPose(target, adjustedRobotPose);
        return distance;
    }

    public List<PhotonPipelineResult> getAllUnreadResults() {
        return photonCamera.getAllUnreadResults();
    }
}