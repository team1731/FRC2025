package frc.robot.subsystems.vision.helpers;

import java.util.List;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.vision.camera.Camera;

public class AprilTagTargetTracker {
    private Camera camera1;
    private Camera camera2;
    private Camera lockedCamera;
    private PhotonTrackedTarget lockedTarget;
    private boolean targetIsVisible;
    private Pose2d goalPose;
    private double calcuatedStrafe;
    private double calcuatedForward;
    private double calcuatedTurn;

    public AprilTagTargetTracker(Camera camera1, Camera camera2) {
        camera1 = this.camera1;
        camera2 = this.camera2;
    }

    public void lockOnTarget(Pose2d currentPose) {
        //List<PhotonTrackedTarget> camera1Targets = getTargets(null);
        //determineClosestTarget(null, currentPose);
    }

    public boolean isLockedOnTarget() {
        return (lockedTarget != null);
    }

    public boolean isTargetVisible() {
        return targetIsVisible;
    }

    /*
     * TODO REFACTOR
     */
    public void recalculateDriveFeedback(Pose2d currentPose) {
        if(lockedTarget == null) return; // no target locked yet

        // See if locked target is visible
        if(!locateLockedTarget()) return; // not currently visible

        /*
         * TODO NEW LOGIC
         */

        // Calculate updated drive values
        calcuatedStrafe = 0; // TODO REPLACE
        calcuatedForward = 0; // TODO REPLACE
        calcuatedTurn = 0; // TODO REPLACE
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

    private boolean locateLockedTarget() {
        List<PhotonTrackedTarget> targets = getTargets(lockedCamera);
        for(var target : targets) {
            if(target.getFiducialId() == lockedTarget.getFiducialId()) {
                targetIsVisible = true;
                lockedTarget = target;
                return true;
            }
        }
        targetIsVisible = false;
        return false;
    }

    /*
     * TODO REFACTOR
     */
    private void determineClosestTarget(List<PhotonTrackedTarget> targets, Pose2d currentPose) {
        PhotonTrackedTarget closestTarget = null;
        double currentTargetDistance = 0;

        for(var target : targets) {
            int targetId = target.getFiducialId();
            if(!FieldPoseHelper.isReefTarget(targetId)) continue;
            /*
             * TODO NEW LOGIC
             */
        }

        if(closestTarget != null) {
            targetIsVisible = true;
            lockedTarget = closestTarget;
        }
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
