package frc.robot.subsystems.vision.helpers;

import java.util.List;

import org.photonvision.targeting.PhotonTrackedTarget;

import frc.robot.subsystems.vision.AprilTagSubsystem;
import frc.robot.subsystems.vision.AprilTagSubsystem.AprilTagTarget;
import frc.robot.subsystems.vision.camera.Camera;

public class AutoLineupHelper {
    public enum LineupPosition {
        LEFT, CENTER, RIGHT
    }

    public enum LineupInstruction {
        TOO_FAR_LEFT, ON_TARGET, TOO_FAR_RIGHT
    }

    private Camera camera1;
    private Camera camera2;
    private Camera lockedCamera;
    private int lockedTargetId;
    private double lastMeasuredYaw;
    private LineupPosition lineupPosition;
    private boolean initialized = false;

    public LineupPosition getLineupPosition() {
        return lineupPosition;
    }

    public void initialize(Camera camera1, Camera camera2) {
        this.camera1 = camera1;
        this.camera2 = camera2;

        determineCameraForLineup();
        if(lockedCamera != null && lockedTargetId != 0 && lineupPosition != null) {
            initialized = true;
        }
    }

    public boolean isInitialized() {
        return initialized;
    }

    public LineupInstruction getLineupFeedback() {
        double measuredYaw = getLockedTargetYaw();
        if(lineupPosition == LineupPosition.LEFT) {
            return checkIfYawWithinTolerance(measuredYaw, FieldPoseHelper.autoLineupLeftTargetYaw);
        } else if(lineupPosition == LineupPosition.RIGHT) {
            return checkIfYawWithinTolerance(measuredYaw, FieldPoseHelper.autoLineupRightTargetYaw);
        } else {
            return checkIfYawWithinTolerance(measuredYaw, FieldPoseHelper.autoLineupCenterTargetYaw);
        }
    }

    private void determineCameraForLineup() {
        Camera winningCamera = null;
        PhotonTrackedTarget winningTarget = null;
        double winningCameraYaw = 0;

        List<AprilTagTarget> targetMap = AprilTagSubsystem.getCombinedTargets(camera1, camera2);
        if(targetMap == null) return;
        System.out.println("target map was not null");

        for(var mapping : targetMap) {
            PhotonTrackedTarget target = mapping.target;
            int targetId = target.getFiducialId();

            if(targetId != FieldPoseHelper.blueAllianceAutoLineupTargetId && 
               targetId != FieldPoseHelper.redAllianceAutoLineupTargetId) continue; // do not see either lineup target
            
            double measuredYaw = target.getYaw();
            boolean newWinningCamera = false;
            if(winningCamera == null) {
                newWinningCamera = true;
            } else {
                LineupPosition position = determineLineupPosition(measuredYaw);
                if((position == LineupPosition.LEFT && measuredYaw > winningCameraYaw) || // for left position, higher yaw is closer to target
                   (position != LineupPosition.LEFT && measuredYaw < winningCameraYaw)) { // for center/right, lower yaw is closer to target
                    newWinningCamera = true;
                }
            }

            if(newWinningCamera) {
                winningCamera = mapping.camera;
                winningTarget = target;
                winningCameraYaw = measuredYaw;
            }
        }

        if(winningCamera != null) {
            lockedCamera = winningCamera;
            lockedTargetId = winningTarget.getFiducialId();
            lastMeasuredYaw = winningCameraYaw;
            lineupPosition = determineLineupPosition(winningCameraYaw);
        }
    }

    private LineupPosition determineLineupPosition(double yaw) {
        if(yaw < FieldPoseHelper.autoLineupLeftPositionThreshold) {
            return LineupPosition.LEFT;
        } else if(yaw > FieldPoseHelper.autoLineupRightPositionThreshold) {
            return LineupPosition.RIGHT;
        } else {
            return LineupPosition.CENTER;
        }
    }

    private double getLockedTargetYaw() {
        List<PhotonTrackedTarget> targets = AprilTagSubsystem.getTargets(lockedCamera);
        if (targets != null) {
        for(var target : targets) {
            if(target.getFiducialId() == lockedTargetId) {
                lastMeasuredYaw = target.getYaw();
                return target.getYaw();
            }
        } 
    }
        return lastMeasuredYaw;
    
    }

    private LineupInstruction checkIfYawWithinTolerance(double measuredYaw, double onTargetYaw) {
        if(measuredYaw < onTargetYaw - FieldPoseHelper.autoLineupTolerance) {
            return LineupInstruction.TOO_FAR_LEFT;
        } else if(measuredYaw > onTargetYaw + FieldPoseHelper.autoLineupTolerance) {
            return LineupInstruction.TOO_FAR_RIGHT;
        }
        return LineupInstruction.ON_TARGET;
    }
}
