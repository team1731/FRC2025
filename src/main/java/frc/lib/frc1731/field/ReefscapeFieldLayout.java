package frc.lib.frc1731.field;

import edu.wpi.first.apriltag.AprilTagFields;

/**
 * Field layout for the 2025 Reefscape game
 */
public class ReefscapeFieldLayout extends FieldLayout {
    public ReefscapeFieldLayout() {
        super(AprilTagLayout.fromYear(AprilTagFields.k2025ReefscapeAndyMark));
    }
}