package frc.robot.subsystems.vision;

import frc.robot.Robot;

public enum ReefTarget {
    A(7, 18, VisionConstants.camera1Name),
    B(7, 18, VisionConstants.camera2Name),
    C(8, 17, VisionConstants.camera1Name),
    D(8, 17, VisionConstants.camera2Name),
    E(9, 22, VisionConstants.camera1Name),
    F(9, 22, VisionConstants.camera2Name),
    G(10, 21, VisionConstants.camera1Name),
    H(10, 21, VisionConstants.camera2Name),
    I(11, 20, VisionConstants.camera1Name),
    J(11, 20, VisionConstants.camera2Name),
    K(6, 19, VisionConstants.camera1Name),
    L(6, 19, VisionConstants.camera2Name);

    private int redAllianceTargetId;
    private int blueAllianceTargetId;
    private String cameraName;
    private ReefTarget(int redAllianceId, int blueAllianceId, String camera) {
        redAllianceTargetId = redAllianceId;
        blueAllianceTargetId = blueAllianceId;
        cameraName = camera;
    }

    public int getTargetId() {
        return Robot.isRedAlliance()? redAllianceTargetId : blueAllianceTargetId;
    }

    public String getCameraName() {
        return cameraName;
    }
}
