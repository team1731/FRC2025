package frc.robot.subsystems.vision.camera;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

public interface Camera {
    public String getName();
    public PhotonCamera getCamera();
    public void initialize();
    public List<PhotonPipelineResult> getAllUnreadResults();
}
