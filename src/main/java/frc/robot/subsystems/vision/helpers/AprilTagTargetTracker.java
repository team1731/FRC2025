package frc.robot.subsystems.vision.helpers;

import java.util.List;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.subsystems.vision.AprilTagSubsystem;
import frc.robot.subsystems.vision.ReefTarget;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.camera.Camera;

public class AprilTagTargetTracker {
    private static AprilTagTargetTracker instance;
    private AprilTagSubsystem aprilTagSubsystem;
    private static ReefTarget currentReefTarget = ReefTarget.A;
    private static Camera lockedCamera;
    private static int lockedTargetId;
    private double lastHeading;
    private double calculatedX;
    private double calculatedY;
    private Rotation2d calculatedDesiredRotation;
    private boolean hasVisibleTarget = false;

    public static AprilTagTargetTracker getTargetTracker(AprilTagSubsystem subsystem) {
        if(instance == null) {
            instance = new AprilTagTargetTracker(subsystem);
        }
        return instance;
    }

    private AprilTagTargetTracker(AprilTagSubsystem subsystem) {
        aprilTagSubsystem = subsystem;
    }

    public static void setReefTarget(ReefTarget reefTarget) {
        System.out.println("AprilTagTargetTracker: changed reef target to " + reefTarget);
        lockedTargetId = 0;
        lockedCamera = null;
        currentReefTarget = reefTarget;
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
           // target = lookForLockedTarget();
            target = chooseTarget();
            lastHeading = 0;
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

       /*  calculate speed
        double fieldCentricSpeed = Math.sqrt(fieldCentricX*fieldCentricX + fieldCentricY*fieldCentricY);
        double fieldCentricHeading = Math.atan(fieldCentricY/fieldCentricX);
        double fieldCentricHeadingError = Math.toRadians(desiredHeading) - fieldCentricHeading;
        double speed  = -fieldCentricSpeed * Math.cos(fieldCentricHeadingError);
       */

        double fieldCentricSpeed = Math.sqrt(fieldCentricX*fieldCentricX + fieldCentricY*fieldCentricY);
        double fieldCentricHeading = Math.atan(fieldCentricY/fieldCentricX);
        double fieldCentricHeadingError = Math.toRadians(desiredHeading) - fieldCentricHeading;
        double speed  = fieldCentricSpeed * Math.cos(fieldCentricHeadingError);


        // calculate updated drive values
        calculatedX = speed * Math.cos(Units.degreesToRadians(desiredHeading));
        calculatedY = speed * Math.sin(Units.degreesToRadians(desiredHeading));
        calculatedDesiredRotation = FieldPoseHelper.getReefTargetLineupRotation(lockedTargetId);
        if (!Robot.isRedAlliance()) {
            calculatedX = calculatedX * -1;
            calculatedY = calculatedY * -1;

        }

        
        //SmartDashboard.putNumber("ATTracker_speedContributionFromX", speedContributionFromX);
        //SmartDashboard.putNumber("ATTracker_speedContributionFromY", speedContributionFromY);
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
        if(aprilTagSubsystem == null) return null;
        
        lockedTargetId = currentReefTarget.getTargetId();
        lockedCamera = currentReefTarget.getCameraName() == VisionConstants.camera1Name? 
            aprilTagSubsystem.getCamera1() : 
            aprilTagSubsystem.getCamera2();

        System.out.println("AprilTagTargetTracker: chose target for post " + 
            currentReefTarget + " Target ID: " + 
            lockedTargetId + " Camera: " + 
            lockedCamera.getName());
        
        return lookForLockedTarget();
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
