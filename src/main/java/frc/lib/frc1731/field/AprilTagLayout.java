package frc.lib.frc1731.field;

import java.util.List;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

/**
 * Class representing the layout of the april tags around a particular game's field
 */
public class AprilTagLayout extends AprilTagFieldLayout {
    public AprilTagLayout(List<AprilTag> apriltags, double fieldLength, double fieldWidth) {
        super(apriltags, fieldLength, fieldWidth);
    }

    public static AprilTagLayout fromYear(AprilTagFields field) {
        AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(field);
        return new AprilTagLayout(fieldLayout.getTags(), fieldLayout.getFieldLength(), fieldLayout.getFieldWidth());
    }
}