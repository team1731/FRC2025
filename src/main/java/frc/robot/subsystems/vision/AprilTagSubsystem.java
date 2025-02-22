
 package frc.robot.subsystems.vision;


import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.ToggleableSubsystem;
import frc.robot.subsystems.vision.camera.BasicCamera;
import frc.robot.subsystems.vision.camera.Camera;

 
 public class AprilTagSubsystem extends SubsystemBase implements ToggleableSubsystem {
    private Camera camera1;
    private Camera camera2;
    private int visionInitCount;
    private boolean enabled;
    private boolean initialized = false;

    public AprilTagSubsystem(boolean enabled) {
        this.enabled = enabled;
        visionInitCount = 0;
        initializeCameras(); // using default processing type
    }
 
    @Override
    public boolean isEnabled() {
        return enabled;
    }

    public Camera getCamera1() {
        return camera1;
    }

    public Camera getCamera2() {
        return camera2;
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
 
    private void initializeCameras() {
        if(camera1 == null) {
            camera1 = new BasicCamera(VisionConstants.camera1Name);
        }

        if(camera2 == null) {
            camera2 = new BasicCamera(VisionConstants.camera2Name);
        }

        if(!camera1.isInitialized()) camera1.initialize();
        if(!camera2.isInitialized()) camera2.initialize();

        if(camera1.isInitialized() || camera2.isInitialized()) {
            initialized = true;
            visionInitCount = 0;
        }
    }
    
    @Override
    public void periodic() {
        if(!initialized) {
            if (visionInitCount++ >= 100) { // 20ms @ 50
                initializeCameras();
                visionInitCount = 0;
            }
        }
    }
 }