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
        TOO_FAR_LEFT, ON_TARGET, TOO_FAR_RIGHT, TARGET_NOT_VISIBLE
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
        if(measuredYaw == -1) {
            return LineupInstruction.TARGET_NOT_VISIBLE;
        } else if(lineupPosition == LineupPosition.LEFT) {
            return checkIfYawWithinTolerance(measuredYaw, FieldPoseHelper.autoLineupLeftTargetYaw);
        } else if(lineupPosition == LineupPosition.RIGHT) {
            return checkIfYawWithinTolerance(measuredYaw, FieldPoseHelper.autoLineupRightTargetYaw);
        } else {
            return checkIfYawWithinTolerance(measuredYaw, FieldPoseHelper.autoLineupCenterTargetYaw);
        }
    }

    private void determineCameraForLineup() {
        Camera winningCamera = null;

        List<AprilTagTarget> targetMap = AprilTagSubsystem.getCombinedTargets(camera1, camera2);
        if(targetMap == null) return;

        for(var mapping : targetMap) {
            PhotonTrackedTarget target = mapping.target;
            int targetId = target.getFiducialId();

            if(targetId != FieldPoseHelper.blueAllianceAutoLineupTargetId && 
               targetId != FieldPoseHelper.redAllianceAutoLineupTargetId) continue; // do not see either lineup target
            
            double measuredYaw = target.getYaw();
            if(winningCamera == null) {
                // For left/center lineup use #1 (ArducamUSB1), otherwise use #2 (ArducamUSB3)
                winningCamera = (measuredYaw > FieldPoseHelper.autoLineupRightPositionThreshold)? camera1 : camera2; 
            }
            
            if(winningCamera != null && mapping.camera.getName() == winningCamera.getName()) {
                // this camera associated with this target is our inside camera
                lockedCamera = winningCamera;
                lockedTargetId = target.getFiducialId();
                lastMeasuredYaw = measuredYaw;
                lineupPosition = determineLineupPosition(measuredYaw);
            }
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
        return -1; // don't see the target anymore
    
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
