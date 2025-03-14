package frc.robot.subsystems.vision.helpers;

import java.util.List;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.subsystems.vision.AprilTagSubsystem;
import frc.robot.subsystems.vision.AprilTagSubsystem.AprilTagTarget;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.camera.Camera;

public class AprilTagTargetTracker {
    private Camera camera1;
    private Camera camera2;
    private Camera lockedCamera;
    private int lockedTargetId;
    private double lastHeading;
    private double calculatedX;
    private double calculatedY;
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
        calculatedX = fieldCentricX;
        calculatedY = fieldCentricY;

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
        double desiredHeading = hasVisibleTarget? currentRobotRotation - (target.getYaw() * 2.0) : lastHeading;
        lastHeading = desiredHeading; // store this value for re-use in case don't see target the next time around

        // calculate speed
      //  double fieldCentricSpeed = Math.sqrt(fieldCentricX*fieldCentricX - fieldCentricY*fieldCentricY);
       // double fieldCentricHeading = Math.atan(fieldCentricY/fieldCentricX);
       // double speed = fieldCentricSpeed * Math.cos(desiredHeading + Math.toRadians(fieldCentricHeading));

        double speedContributionFromX = fieldCentricX * Math.cos(Units.degreesToRadians(desiredHeading));
        double speedContributionFromY = fieldCentricY * Math.sin(Units.degreesToRadians(desiredHeading));
        double speed = -Math.sqrt(speedContributionFromX*speedContributionFromX + speedContributionFromY*speedContributionFromY);


        // calculate updated drive values
        calculatedX = speed * Math.cos(Units.degreesToRadians(desiredHeading));
        calculatedY = speed * Math.sin(Units.degreesToRadians(desiredHeading));
        calculatedDesiredRotation = FieldPoseHelper.getReefTargetLineupRotation(lockedTargetId);
        if (!Robot.isRedAlliance()) {
            calculatedX = calculatedX * -1;
            calculatedY = calculatedY * -1;

        }

        
      //  SmartDashboard.putNumber("ATTracker_speedContributionFromX", fieldCentricSpeed);
      //  SmartDashboard.putNumber("ATTracker_speedContributionFromY", fieldCentricHeading);
        SmartDashboard.putNumber("ATTracker_totalSpeed", speed);
        SmartDashboard.putNumber("ATTracker_targetedAprilTagId", lockedTargetId);
        SmartDashboard.putNumber("ATTracker_calculatedX", calculatedX);
        SmartDashboard.putNumber("ATTracker_calculatedY", calculatedY);
        SmartDashboard.putNumber("ATTracker_rotation", calculatedDesiredRotation.getDegrees());
        SmartDashboard.putNumber("ATTracker_currentRobotRotation", currentRobotRotation);
        SmartDashboard.putNumber("ATTracker_targetYaw", hasVisibleTarget? target.getYaw() : 0);
        SmartDashboard.putNumber("ATTracker_desiredHeading", desiredHeading);
        
    }

    public double getCalculatedX() {
        return calculatedX;
    }

    public double getCalculatedY() {
        return calculatedY;
    }

    public Rotation2d getRotationTarget() {
        return calculatedDesiredRotation;
    }
    
    private PhotonTrackedTarget chooseTarget() {
        Camera winningCamera = null;
        PhotonTrackedTarget winningTarget = null;
        double winningTargetSize = 0;

        List<AprilTagTarget> targetMap = AprilTagSubsystem.getCombinedTargets(camera1, camera2);
        if(targetMap == null) return null;

        for(var mapping : targetMap) {
            PhotonTrackedTarget target = mapping.target;
            int targetId = target.getFiducialId();

            if(!FieldPoseHelper.isReefTarget(targetId)) continue;
            double fovRatio = (mapping.camera.getName() == VisionConstants.camera2Name)? VisionConstants.camera2FOVRatio : 1.0;
            double targetSize = Math.abs(target.getArea()) * fovRatio;
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

    private PhotonTrackedTarget lookForLockedTarget() {
        List<PhotonTrackedTarget> targets = AprilTagSubsystem.getTargets(lockedCamera);
        if(targets != null) {
            for(var target : targets) {
                if(target.getFiducialId() == lockedTargetId) return target;
            }
        }
        return null;
    }
}
