package frc.robot.subsystems.vision.helpers;

import java.util.List;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.camera.Camera;

public class AprilTagTargetTracker {
    private Camera camera1;
    private Camera camera2;
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
        PhotonTrackedTarget target = chooseTarget();
        if(target == null) {
            hasVisibleTarget = false;
            return;
        }  
        
        hasVisibleTarget = true;


        var currentRobotRotation = currentPose.getRotation().getDegrees();
        double desiredHeading = currentRobotRotation + target.getYaw();


        // calculate speed
   
        double speedContributionFromX = fieldCentricX * Math.cos(desiredHeading);
        double speedContributionFromY = fieldCentricY * Math.sin(desiredHeading);
        double speed = speedContributionFromX + speedContributionFromY;


        // calculate updated drive values

        calcuatedForward = speed * Math.cos(Units.degreesToRadians(desiredHeading));
        calcuatedStrafe = speed * Math.sin(Units.degreesToRadians(desiredHeading));
        calculatedDesiredRotation = FieldPoseHelper.getDriveToTagRotation(target.getFiducialId());
        

        
        SmartDashboard.putNumber("ATTracker_speedContributionFromX", speedContributionFromX);
        SmartDashboard.putNumber("ATTracker_speedContributionFromY", speedContributionFromY);
        SmartDashboard.putNumber("ATTracker_totalSpeed", speed);
        SmartDashboard.putNumber("ATTracker_targetedAprilTagId", target.getFiducialId());
        SmartDashboard.putNumber("ATTracker_forward", calcuatedForward);
        SmartDashboard.putNumber("ATTracker_strafe", calcuatedStrafe);
        SmartDashboard.putNumber("ATTracker_rotation", calculatedDesiredRotation.getDegrees());
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
        PhotonTrackedTarget winningTarget = null;
        double winningTargetYaw = 0;

        List<PhotonTrackedTarget> targets = getCombinedTargets();
        if(targets == null) return null;

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

    private double limitNegToPosOne(double value) {
        var result = Math.min(value, 1.0);
        return Math.max(result,-1);
    }


}
