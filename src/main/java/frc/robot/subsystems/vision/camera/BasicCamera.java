package frc.robot.subsystems.vision.camera;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;


public class BasicCamera implements Camera {
    private String name;
    private PhotonCamera photonCamera;

    public BasicCamera(String cameraName) {
        name = cameraName;
    }

    public void initialize() {
        // no initialization required for the basic camera
    }

    public String getName() {
        return name;
    }

    public PhotonCamera getCamera() {
        return photonCamera;
    }

    public List<PhotonPipelineResult> getAllUnreadResults() {
        return photonCamera.getAllUnreadResults();
    }
}
