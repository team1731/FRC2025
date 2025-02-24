package frc.robot.subsystems.vision.camera;

import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;


public class BasicCamera implements Camera {
    private String name;
    private PhotonCamera photonCamera;
    private boolean initialized = false;

    public BasicCamera(String cameraName) {
        name = cameraName;
    }

    public void initialize() {
        if(initialized) return;
        
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable photonVisionTable = inst.getTable("photonvision/" + name);
        if(photonVisionTable.containsKey("hasTarget")) {
            photonCamera = new PhotonCamera(name);
            initialized = true;
            System.out.println("Camera: Adding basic camera " + name + "!!!!!!! ");
        } else {
            System.out.println("VisionSubsystem: Init FAILED: " + " Keys: " + photonVisionTable.getKeys().toString());
        }
    }

    public boolean isInitialized() {
        return initialized;
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
