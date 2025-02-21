
 package frc.robot.subsystems.vision;


import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.ToggleableSubsystem;
import frc.robot.subsystems.vision.camera.BasicCamera;
import frc.robot.subsystems.vision.camera.Camera;
import frc.robot.subsystems.vision.camera.CameraType;
import frc.robot.subsystems.vision.camera.PNPCamera;

 
 public class AprilTagSubsystem extends SubsystemBase implements ToggleableSubsystem {
    private Camera camera;
    private CameraType cameraType = CameraType.BASIC; //default
    private int visionInitCount;
    private boolean enabled;
    private boolean initialized;

    public AprilTagSubsystem(boolean enabled) {
        this.enabled = enabled;
        visionInitCount = 0;
        initializeCamera(); // default camera
    }
 
    @Override
    public boolean isEnabled() {
        return enabled;
    }

    public Camera getCamera() {
        return camera;
    }

    public void setCameraType(CameraType type) {
        cameraType = type;
    }
 
    public static void setupPortForwarding() {
        PortForwarder.add(5800, "photonvision.local", 5800);
        PortForwarder.add(1181, "photonvision.local", 1181);
        PortForwarder.add(1182, "photonvision.local", 1182);
        PortForwarder.add(1183, "photonvision.local", 1183);
        PortForwarder.add(1184, "photonvision.local", 1184);
        PortForwarder.add(1185, "photonvision.local", 1185);
        PortForwarder.add(1186, "photonvision.local", 1186);
        PortForwarder.add(1187, "photonvision.local", 1187);
    }
 
    public void initializeCamera() {
        initialized = false; 
        camera = null;
        String cameraName = VisionConstants.macDyverCamera;
        
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        // get the subtable called "photonvision"
        NetworkTable photonVisionTable = inst.getTable("photonvision/" + cameraName);
        if (photonVisionTable.containsKey("hasTarget")) {
            camera = cameraType == CameraType.BASIC? new BasicCamera(cameraName) : new PNPCamera(cameraName, VisionConstants.robotToCamera3d);
            camera.initialize();
            initialized = true;
            System.out.println("VisionSubsystem: Adding camera " + cameraName + "!!!!!!! ");
        }

        if (!initialized) {
            System.out.println("VisionSubsystem: Init FAILED: " + " Keys: " + photonVisionTable.getKeys().toString());
        }
    }
    
    @Override
    public void periodic() {
        if (!initialized) {
            // System.out.println("Checking vision, currently not initialized");
            if (visionInitCount++ >= 100) { // 20ms @ 50
                initializeCamera();
                visionInitCount = 0;
            }
        }
    }
 }