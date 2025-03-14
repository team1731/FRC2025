
 package frc.robot.subsystems.vision;


import java.util.ArrayList;
import java.util.List;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.ToggleableSubsystem;
import frc.robot.subsystems.leds.LEDSubsystem;
import frc.robot.subsystems.vision.camera.BasicCamera;
import frc.robot.subsystems.vision.camera.Camera;
import frc.robot.subsystems.vision.camera.CameraChoice;
import frc.robot.subsystems.vision.helpers.AutoLineupHelper;
import frc.robot.subsystems.vision.helpers.AutoLineupHelper.LineupInstruction;

 
 public class AprilTagSubsystem extends SubsystemBase implements ToggleableSubsystem {
    public static class AprilTagTarget {
        public Camera camera;
        public PhotonTrackedTarget target;
    }

    private Camera camera1;
    private Camera camera2;
    private int visionInitCount;
    private boolean enabled;
    private boolean initialized = false;
    private LEDSubsystem ledSubsystem;
    private AutoLineupHelper autoLineupHelper;


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

    public void setLEDSubsystem(LEDSubsystem ledSubsystem) {
        this.ledSubsystem = ledSubsystem;
    }

    public void startAutoLineup() {
        autoLineupHelper = new AutoLineupHelper();
    }

    public void stopAutoLineup() {
        autoLineupHelper = null;
        ledSubsystem.turnLineupColorsOff();
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

    public static List<PhotonTrackedTarget> getTargets(Camera camera) {
        if(camera == null || !camera.isInitialized()) return null;

        List<PhotonPipelineResult> results = camera.getAllUnreadResults();
        if(results.size() > 0) {
            var result = results.get(results.size() - 1); // Camera processed a new frame since last, get the last one in the list
            if(result.hasTargets()) {
                return result.getTargets();
            }
        }
        return null;
    }

    public static List<AprilTagTarget> getCombinedTargets(Camera camera1, Camera camera2) {
        List<AprilTagTarget> combinedTargets = new ArrayList<AprilTagTarget>();

        List<PhotonTrackedTarget> camera1Targets = AprilTagSubsystem.getTargets(camera1);
        if(camera1Targets != null) {
            mapTargets(combinedTargets, camera1, camera1Targets);
        }
        if (camera2 != null) {
            List<PhotonTrackedTarget> camera2Targets = AprilTagSubsystem.getTargets(camera2);
            if(camera2Targets != null) {
                mapTargets(combinedTargets, camera2, camera2Targets);
            }
        }

        return (combinedTargets.size() > 0)? combinedTargets : null;
    }

    private static List<AprilTagTarget> mapTargets(List<AprilTagTarget> map, Camera camera, List<PhotonTrackedTarget> targets) {
        for(var target : targets) {
            AprilTagTarget mapping = new AprilTagTarget();
            mapping.camera = camera;
            mapping.target = target;
            map.add(mapping);
        }
        return map;
    }
    
    @Override
    public void periodic() {
        if(!initialized) {
            if (visionInitCount++ >= 100) { // 20ms @ 50
                initializeCameras();
                visionInitCount = 0;
            }
        }

        if(initialized && autoLineupHelper != null) {
            if(!autoLineupHelper.isInitialized()) {
                autoLineupHelper.initialize(camera1, camera2);
            }

            if(autoLineupHelper.isInitialized()) {
                LineupInstruction lineupInstruction = autoLineupHelper.getLineupFeedback();
                if(lineupInstruction == LineupInstruction.TARGET_NOT_VISIBLE) {
                    ledSubsystem.setNoTagFound();
                } else if(lineupInstruction == LineupInstruction.TOO_FAR_LEFT) {
                    ledSubsystem.setLineupTooFarLeftScheme();
                    SmartDashboard.putString("Operator Command too far:","Left");
                } else if(lineupInstruction == LineupInstruction.TOO_FAR_RIGHT) {
                    ledSubsystem.setLineupTooFarRightScheme();
                    SmartDashboard.putString("Operator Command too far: ","RIGHT");
                } else {
                    ledSubsystem.setLineupCenteredScheme();
                    SmartDashboard.putString("Operator Command too far: ","Just Right");
                }

                
            }
        }
    }

    public Camera getCamera(CameraChoice m_cameraChoice) {
        if (m_cameraChoice == CameraChoice.ElevSide) {
            return camera1;
        } else {
            return camera2;
        }
    }
 }