package frc.lib.frc1731.field;

import java.io.IOException;
import java.util.List;

import edu.wpi.first.apriltag.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Class that holds useful information for a particular game's field
 */
public abstract class FieldLayout {
    private AprilTagLayout aprilTagLayout;
    private Field2d field = new Field2d();

    public enum Alliance {
        kRed,
        kBlue;
    }

    public FieldLayout(AprilTagLayout layout) {
        this.aprilTagLayout = layout;
    }

    public FieldLayout(List<AprilTag> tags, double fieldLength, double fieldWidth) {
        this.aprilTagLayout = new AprilTagLayout(tags, fieldLength, fieldWidth);
    }

    public AprilTagFieldLayout getAprilTagLayout() {
        return this.aprilTagLayout;
    }

    public Pose2d getSimulatedRobotPose() {
        return this.field.getRobotPose();
    }

    public void setSimulatedRobotPose(Pose2d robotPose) {
        this.field.setRobotPose(robotPose);
    }

    public Field2d getField() {
        return this.field;
    }

    public double getFieldLength() {
        return this.aprilTagLayout.getFieldLength();
    }

    public double getFieldWidth() {
        return this.aprilTagLayout.getFieldWidth();
    }

    public void logToShuffleboard(boolean log) {
        if (log) SmartDashboard.putData("Field", field);
    }

    public static FieldLayout none() {
        return new FieldLayout(new AprilTagLayout(List.of(), 16.542d, 8d)) {};
    }

    public static AprilTagFieldLayout createLayout(String path) {
        try {
            return AprilTagFieldLayout.loadFromResource(path);
        } catch (IOException e) {
            System.out.println("April Tag Layout Does Not Exist!!");
            return null;
        }
    }
}