package frc.robot.subsystems.vision.helpers;

import java.util.List;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.vision.AprilTagSubsystem;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.AprilTagSubsystem.AprilTagTarget;
import frc.robot.subsystems.vision.camera.Camera;

public class WorkingAprilTagTargetTracker {
    private Camera camera1;
    private Camera camera2;
    private Camera lockedCamera;
    private int lockedTargetId;
    private double lastHeading;
    private double calcuatedX;
    private double calcuatedY;
    private Rotation2d calculatedDesiredRotation;
    private boolean hasVisibleTarget = false;

    public WorkingAprilTagTargetTracker(Camera camera1, Camera camera2) {
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

        Rotation2d currentRobotRotation = currentPose.getRotation();
        Rotation2d lineupRotation = FieldPoseHelper.getReefTargetLineupRotation(lockedTargetId);

        double desiredHeading;
        if(hasVisibleTarget) {
            double targetYaw = target.getYaw();
            desiredHeading = getDesiredHeading(targetYaw, currentRobotRotation, lineupRotation);
            lastHeading = desiredHeading; // store this value for re-use in case don't see target the next time around
        } else {
            desiredHeading = lastHeading;
        }

        // calculate speed
        double speedContributionFromX = fieldCentricX * Math.cos(Units.degreesToRadians(desiredHeading));
        double speedContributionFromY = fieldCentricY * Math.sin(Units.degreesToRadians(desiredHeading));
        double speed = -Math.sqrt(speedContributionFromX*speedContributionFromX + speedContributionFromY*speedContributionFromY);


        // calculate updated drive values
        calcuatedX = speed * Math.cos(Units.degreesToRadians(desiredHeading));
        calcuatedY = speed * Math.sin(Units.degreesToRadians(desiredHeading));
        calculatedDesiredRotation = lineupRotation;
        

        
        SmartDashboard.putNumber("ATTracker_speedContributionFromX", speedContributionFromX);
        SmartDashboard.putNumber("ATTracker_speedContributionFromY", speedContributionFromY);
        SmartDashboard.putNumber("ATTracker_totalSpeed", speed);
        SmartDashboard.putNumber("ATTracker_targetedAprilTagId", lockedTargetId);
        SmartDashboard.putNumber("ATTracker_calcX", calcuatedX);
        SmartDashboard.putNumber("ATTracker_calcY", calcuatedY);
        SmartDashboard.putNumber("ATTracker_calcRotation", calculatedDesiredRotation.getDegrees());
        SmartDashboard.putNumber("ATTracker_currentRobotRotation", currentPose.getRotation().getDegrees());
        SmartDashboard.putNumber("ATTracker_targetYaw", hasVisibleTarget? target.getYaw() : 0);
        SmartDashboard.putNumber("ATTracker_desiredHeading", desiredHeading);
        
    }

    public double getCalcuatedX() {
        return calcuatedX;
    }

    public double getCalculatedY() {
        return calcuatedY;
    }

    public Rotation2d getRotationTarget() {
        return calculatedDesiredRotation;
    }

    private double getDesiredHeading(double targetYaw, Rotation2d robotRotation, Rotation2d lineupRotation) {
        double modifiedYaw = targetYaw * 2.0;
        double robotRotationDegrees = robotRotation.getDegrees();
        double calculatedHeading = robotRotationDegrees - modifiedYaw;
        double lowerConstraint = lineupRotation.getDegrees() - 90;
        double upperConstraint = lineupRotation.getDegrees() + 90;
        //if(calculatedHeading < lowerConstraint) return lowerConstraint;
        //else if(calculatedHeading > upperConstraint) return upperConstraint;
        //else return calculatedHeading;
        return calculatedHeading;
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
