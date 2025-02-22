package frc.robot.subsystems.vision.helpers;

import java.util.List;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.camera.Camera;

public class AprilTagTargetTracker {
    private Camera camera1;
    private Camera camera2;
    private double calcuatedStrafe;
    private double calcuatedForward;
    private double calcuatedTurn;
    private boolean hasVisibleTarget = false;

    public AprilTagTargetTracker(Camera camera1, Camera camera2) {
        camera1 = this.camera1;
        camera2 = this.camera2;
    }

    public boolean hasVisibleTarget() {
        return hasVisibleTarget;
    }

    public void recalculateDriveFeedback(Pose2d currentPose, double fieldCentricX, double fieldCentricY) {
        PhotonTrackedTarget target = chooseTarget();
        if(target == null) {
            hasVisibleTarget = false;
            return;
        }

        hasVisibleTarget = true;
        SmartDashboard.putNumber("ATTracker_targetedAprilTagId", target.getFiducialId());

        // calculate speed
        var tagRotation = 180.0;
        var speedContributionFromX = fieldCentricX * Math.sin(Units.degreesToRadians(tagRotation));
        var speedContributionFromY = fieldCentricY * Math.cos(Units.degreesToRadians(tagRotation));
        var speed = speedContributionFromX + speedContributionFromY;
        SmartDashboard.putNumber("ATTracker_speedContributionFromX", speedContributionFromX);
        SmartDashboard.putNumber("ATTracker_speedContributionFromY", speedContributionFromY);

        // calculate updated drive values
        calcuatedForward = speed * Math.cos(Units.degreesToRadians(target.getYaw()));
        calcuatedStrafe = speed * Math.sin(Units.degreesToRadians(target.getYaw()));
        calcuatedTurn = limitNegToPosOne((currentPose.getRotation().getDegrees() - tagRotation) * VisionConstants.VISION_ROTATE_kP) * VisionConstants.MAX_ANGULAR_SPEED;
        SmartDashboard.putNumber("ATTracker_forward", calcuatedForward);
        SmartDashboard.putNumber("ATTracker_strafe", calcuatedStrafe);
        SmartDashboard.putNumber("ATTracker_turn", calcuatedTurn);
    }

    public double getCalculatedStrafe() {
        return calcuatedStrafe;
    }

    public double getCalcuatedForward() {
        return calcuatedForward;
    }

    public double getCalcuatedTurn() {
        return calcuatedTurn;
    }
    
    private PhotonTrackedTarget chooseTarget() {
        PhotonTrackedTarget winningTarget = null;
        double winningTargetYaw = 0;

        List<PhotonTrackedTarget> targets = getCombinedTargets();
        for(var target : targets) {
            int targetId = target.getFiducialId();
            if(!FieldPoseHelper.isReefTarget(targetId)) continue;
            double targetYaw = Math.abs(target.getYaw());
            if(winningTarget == null || targetYaw < winningTargetYaw) {
                winningTarget = target;
                winningTargetYaw = targetYaw;
            }
        }

        return winningTarget;
    }

    private List<PhotonTrackedTarget> getCombinedTargets() {
        List<PhotonTrackedTarget> combinedTargets;
        List<PhotonTrackedTarget> camera1Targets = getTargets(camera1);
        if(camera1Targets != null) {
            combinedTargets = camera1Targets;
            List<PhotonTrackedTarget> camera2Targets = getTargets(camera2);
            if(camera2Targets != null) combinedTargets.addAll(camera2Targets);
        } else {
            combinedTargets = getTargets(camera2);
        }
        return combinedTargets;
    }
    
    private List<PhotonTrackedTarget> getTargets(Camera camera) {
        List<PhotonPipelineResult> results = camera.getAllUnreadResults();
        var result = results.get(results.size() - 1); // Camera processed a new frame since last, get the last one in the list
        if(result.hasTargets()) {
            return result.getTargets();
        }
        return null;
    }

    private double limitNegToPosOne(double value) {
        var result = Math.min(value, 1.0);
        return Math.max(result,-1);
    }
}
