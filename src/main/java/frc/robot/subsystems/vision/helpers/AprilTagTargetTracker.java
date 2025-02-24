package frc.robot.subsystems.vision.helpers;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.vision.camera.Camera;

public class AprilTagTargetTracker {
    public class AprilTagTarget {
        Camera camera;
        PhotonTrackedTarget target;
    }

    private Camera camera1;
    private Camera camera2;
    private Camera lockedCamera;
    private int lockedTargetId;
    private double lastHeading;
    private double calcuatedStrafe;
    private double calcuatedForward;
    private Rotation2d calculatedDesiredRotation;
    private boolean hasVisibleTarget = false;

    public AprilTagTargetTracker(Camera camera1, Camera camera2) {
        this.camera1 = camera1;
        this.camera2 = camera2;
    }

    public boolean hasVisibleTarget() {
        return hasVisibleTarget;
    }

    public void recalculateDriveFeedback(Pose2d currentPose, double fieldCentricX, double fieldCentricY) {

        SmartDashboard.putNumber("fcx", fieldCentricX);
        SmartDashboard.putNumber("fcy",fieldCentricY);

        PhotonTrackedTarget target;
        if(lockedTargetId != 0) { 
            target = lookForLockedTarget();
        } else {
            target = chooseTarget();
        }

        if(target == null) {
            hasVisibleTarget = false;
        } else {
            hasVisibleTarget = true;
        }

        if(!hasVisibleTarget && lastHeading == 0) {
            return; // haven't captured a heading yet, nothing to re-use
        }

        var currentRobotRotation = currentPose.getRotation().getDegrees();
        double desiredHeading = hasVisibleTarget? currentRobotRotation - target.getYaw() : lastHeading;
        lastHeading = desiredHeading; // store this value for re-use in case don't see target the next time around

        // calculate speed
        double speedContributionFromX = fieldCentricX * Math.cos(Units.degreesToRadians(desiredHeading));
        double speedContributionFromY = fieldCentricY * Math.sin(Units.degreesToRadians(desiredHeading));
        double speed = speedContributionFromX + speedContributionFromY;


        // calculate updated drive values
        calcuatedForward = speed * Math.cos(Units.degreesToRadians(desiredHeading));
        calcuatedStrafe = speed * Math.sin(Units.degreesToRadians(desiredHeading));
        calculatedDesiredRotation = FieldPoseHelper.getReefTargetLineupRotation(lockedTargetId);
        

        
        SmartDashboard.putNumber("ATTracker_speedContributionFromX", speedContributionFromX);
        SmartDashboard.putNumber("ATTracker_speedContributionFromY", speedContributionFromY);
        SmartDashboard.putNumber("ATTracker_totalSpeed", speed);
        SmartDashboard.putNumber("ATTracker_targetedAprilTagId", lockedTargetId);
        SmartDashboard.putNumber("ATTracker_forward", calcuatedForward);
        SmartDashboard.putNumber("ATTracker_strafe", calcuatedStrafe);
        SmartDashboard.putNumber("ATTracker_rotation", calculatedDesiredRotation.getDegrees());
        SmartDashboard.putNumber("ATTracker_currentRobotRotation", currentRobotRotation);
        SmartDashboard.putNumber("ATTracker_targetYaw", hasVisibleTarget? target.getYaw() : 0);
        SmartDashboard.putNumber("ATTracker_desiredHeading", desiredHeading);
        
    }

    public double getCalculatedStrafe() {
        return calcuatedStrafe;
    }

    public double getCalcuatedForward() {
        return calcuatedForward;
    }

    public Rotation2d getRotationTarget() {
        return calculatedDesiredRotation;
    }
    
    private PhotonTrackedTarget chooseTarget() {
        Camera winningCamera = null;
        PhotonTrackedTarget winningTarget = null;
        double winningTargetSize = 0;

        List<AprilTagTarget> targetMap = getCombinedTargets();
        if(targetMap == null) return null;

        for(var mapping : targetMap) {
            PhotonTrackedTarget target = mapping.target;
            int targetId = target.getFiducialId();

            if(!FieldPoseHelper.isReefTarget(targetId)) continue;
            double targetSize = Math.abs(target.getArea());
            if(winningTarget == null || targetSize > winningTargetSize) {
                winningCamera = mapping.camera;
                winningTarget = target;
                winningTargetSize = targetSize;
            }
        }

        if(winningTarget != null) {
            lockedCamera = winningCamera;
            lockedTargetId = winningTarget.getFiducialId();
        }

        return winningTarget;
    }

    private List<AprilTagTarget> getCombinedTargets() {
        List<AprilTagTarget> combinedTargets = new ArrayList<AprilTagTarget>();

        List<PhotonTrackedTarget> camera1Targets = getTargets(camera1);
        if(camera1Targets != null) {
            mapTargets(combinedTargets, camera1, camera1Targets);
        }

        List<PhotonTrackedTarget> camera2Targets = getTargets(camera2);
        if(camera2Targets != null) {
            mapTargets(combinedTargets, camera2, camera2Targets);
        }

        return (combinedTargets.size() > 0)? combinedTargets : null;
    }

    private List<AprilTagTarget> mapTargets(List<AprilTagTarget> map, Camera camera, List<PhotonTrackedTarget> targets) {
        for(var target : targets) {
            AprilTagTarget mapping = new AprilTagTarget();
            mapping.camera = camera;
            mapping.target = target;
            map.add(mapping);
        }
        return map;
    }

    private PhotonTrackedTarget lookForLockedTarget() {
        List<PhotonTrackedTarget> targets = getTargets(lockedCamera);
        if(targets != null) {
            for(var target : targets) {
                if(target.getFiducialId() == lockedTargetId) return target;
            }
        }
        return null;
    }
    
    private List<PhotonTrackedTarget> getTargets(Camera camera) {
        if(camera == null || !camera.isInitialized()) return null;

        List<PhotonPipelineResult> results = camera.getAllUnreadResults();
        if(results.size() > 0) {
            var result = results.get(results.size() - 1); // Camera processed a new frame since last, get the last one in the list
            if(result.hasTargets()) {
                return result.getTargets();
            }
        }
        return null;
    }
}
