package frc.robot.subsystems.vision.helpers;

import java.util.List;

import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.camera.BasicCamera;
import frc.robot.subsystems.vision.camera.Camera;

public class AprilTagTargetTracker {
    private Camera camera;
    private PhotonTrackedTarget lockedTarget;
    private double distanceToTarget;
    private boolean targetIsVisible;
    private Pose2d goalPose;
    private double calcuatedStrafe;
    private double calcuatedForward;
    private double calcuatedTurn;

    public AprilTagTargetTracker() {
        camera = new BasicCamera(VisionConstants.macDyverCamera);
    }

    public void lockOnTarget(Pose2d currentPose) {
        List<PhotonTrackedTarget> targets = getTargets();
        determineClosestTarget(targets, currentPose);
    }

    public boolean isLockedOnTarget() {
        return (lockedTarget != null);
    }

    public boolean isTargetVisible() {
        return targetIsVisible;
    }

    public double getDistanceToTarget() {
        return distanceToTarget;
    }

    public boolean isAtTarget() {
        if(lockedTarget == null) return false;
        double currentDistance = getDistanceToTarget(lockedTarget);
        double tolerance = 0.025; // TODO what should this be?
        return currentDistance <= tolerance;
    }

    public void recalculateDriveFeedback(Pose2d currentPose) {
        if(lockedTarget == null) return;
        double tagRotation = 0;

        // See if locked target is visible
        if(!locateLockedTarget()) return; // not currently visible

        // Target error and yaw calculations
        double rotationError = currentPose.getRotation().getDegrees() - tagRotation;
        double targetYaw = lockedTarget.getYaw() - rotationError;
        double targetRange = getDistanceToTarget(lockedTarget);
        distanceToTarget = targetRange; // update our stored distance
        SmartDashboard.putNumber("AprilTagTargetHelper: rotation error", rotationError);

        // Pose calculations
        Pose2d relativeCameraPose = new Pose2d(-targetRange * Math.cos(targetYaw),
            targetRange * Math.sin(targetYaw), new Rotation2d()); // relative to apriltag 0,0
        Pose2d relativeRobotPose = relativeCameraPose.plus(VisionConstants.robotToCamera2d.inverse()); // relative to apriltag 0,0
        double robotToTargetYaw = Math.toDegrees(Math.atan(relativeRobotPose.getY()/relativeRobotPose.getX()));
        SmartDashboard.putNumber("robottotagYaw", robotToTargetYaw);
        
        // Select the closest goal pose if not already selected
        if(goalPose == null) { 
            goalPose = (robotToTargetYaw < 0)? VisionConstants.leftReefScorePose : VisionConstants.rightReefScorePose;
        }

        // Calculate where the robot is relative to the selected goal
        Transform2d poseRelativeToGoal = relativeRobotPose.minus(goalPose);
        var robotToGoalYaw = Math.toDegrees(Math.atan(relativeRobotPose.getY()/relativeRobotPose.getX()));
        SmartDashboard.putNumber("robottogoalYaw", robotToGoalYaw);
        var robotToGoalDistance = Math.sqrt(poseRelativeToGoal.getX()*poseRelativeToGoal.getX() + poseRelativeToGoal.getY() * poseRelativeToGoal.getY());
        SmartDashboard.putNumber("robtotogoalDistance", robotToGoalDistance);
        double lateralError = robotToGoalDistance*Math.sin(Units.degreesToRadians(rotationError));

        // Calculate updated drive values
        calcuatedStrafe = limitNegToPosOne(lateralError*VisionConstants.VISION_STRAFE_kP) * VisionConstants.MAX_LINEAR_SPEED;
        calcuatedForward = limitNegToPosOne(robotToGoalDistance*VisionConstants.VISION_FORWARD_kP) * VisionConstants.MAX_LINEAR_SPEED;
        calcuatedTurn = limitNegToPosOne((currentPose.getRotation().getDegrees() - goalPose.getRotation().getDegrees()) * VisionConstants.VISION_ROTATE_kP) *VisionConstants. MAX_ANGULAR_SPEED;
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
        List<PhotonTrackedTarget> targets = getTargets();
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


    private void determineClosestTarget(List<PhotonTrackedTarget> targets, Pose2d currentPose) {
        PhotonTrackedTarget closestTarget = null;
        double currentTargetDistance = 0;

        for(var target : targets) {
            int targetId = target.getFiducialId();
            if(!FieldPoseHelper.isReefTarget(targetId)) continue;
            if(closestTarget == null) {
                closestTarget = target;
                currentTargetDistance = getDistanceToTarget(target);
            } else {
                double compareDistance = getDistanceToTarget(target);
                if(compareDistance < currentTargetDistance) {
                    closestTarget = target;
                    currentTargetDistance = getDistanceToTarget(target);
                }
            }
        }

        if(closestTarget != null) {
            targetIsVisible = true;
            lockedTarget = closestTarget;
            distanceToTarget = currentTargetDistance;
        }
    }

    private double getDistanceToTarget(PhotonTrackedTarget target) {
        return PhotonUtils.calculateDistanceToTargetMeters(
            Units.inchesToMeters(VisionConstants.cameraHeight),
            Units.inchesToMeters(FieldPoseHelper.reefAprilTagHeight),
            Units.degreesToRadians(VisionConstants.cameraPitch),
            Units.degreesToRadians(target.getPitch())
        );
    }
    
    private List<PhotonTrackedTarget> getTargets() {
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
