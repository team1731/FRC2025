package frc.robot;

import java.io.File;
import java.util.*;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.AutoConstants;

import static frc.robot.RobotContainer.*;

public class AutoLoader {
    private static SendableChooser<String> autoChooser = new SendableChooser<>();
    private static HashMap<String, String> autoPaths;

    private static boolean isVSLAMConnected = false;
    private static boolean isRedAlliance = false;
    private static boolean flipForRed = false;

    private static String selectedAutoName = "";

    private PathPlannerAuto selectedPPAuto = null;

    public AutoLoader() {
        String[] autoModes = getAutoModes();
		for (String autoMode : autoModes) {
			autoChooser.addOption(autoMode, autoMode);
		}
		
		// pre-load the default auto
        String defaultAuto = Constants.AutoConstants.kAutoDefault;
		autoChooser.setDefaultOption(defaultAuto, defaultAuto);

        // Put on SmartDashboard
        SmartDashboard.putData(AutoConstants.kAutoCodeKey, autoChooser);
    }

    public static boolean flipForRed() {
        return flipForRed;
    }

    private static String[] getAutoModes() {
        autoPaths = findPaths(new File(Filesystem.getLaunchDirectory(),
            (Robot.isReal() ? "home/lvuser" : "src/main") + "/deploy/pathplanner/autos"));
        List<String> autoModes = new ArrayList<String>();
        for (String key : autoPaths.keySet()) {
            String stripKey = key.toString();
            if (stripKey.endsWith(AutoConstants.kNoVSLAMPostfix)) {
                continue; // exclude these from the chooser
            }
            if (stripKey.startsWith("Red_") || stripKey.startsWith("Blu_")) {
                stripKey = stripKey.substring(4, stripKey.length());
            }
            if (!autoModes.contains(stripKey)) {
                autoModes.add(stripKey);
            }
        }
        autoModes.sort((p1, p2) -> p1.compareTo(p2));
        return autoModes.toArray(String[]::new);
    }

    private static HashMap<String, String> findPaths(File directory) {
        HashMap<String, String> autoPaths = new HashMap<String, String>();
        if (!directory.exists()) {
            System.out.println("FATAL: path directory not found! " + directory.getAbsolutePath());
        } else {
            File[] files = directory.listFiles();
            if (files == null) {
                System.out.println("FATAL: I/O error or NOT a directory: " + directory);
            } else {
                for (File file : files) {
                    String fileName = file.getName();
                    if ((fileName.startsWith("Blu") || fileName.startsWith("Red")) && fileName.endsWith(".auto")) {
                        String key = fileName.replace(".auto", "");
                        String path = file.getAbsolutePath();
                        System.out.println(path);
                        autoPaths.put(key, path);
                    }
                }
            }
        }
        return autoPaths;
    }

    private Command autoPreloadCommand() {
        return new InstantCommand(() -> swerve.resetPose(selectedPPAuto.getStartingPose()))
        .onlyIf(() -> Robot.isSimulation())
        .andThen(
            climb.stowCommand()
        );
    }

    public Command getSelectedAutoName() {
        return autoPreloadCommand().andThen(selectedPPAuto);
    }

    public void update() {
        isVSLAMConnected = swerve.getVSLAMSubsystem().isConnected();
        isRedAlliance = Robot.isRedAlliance();

        String auto = autoChooser.getSelected();

        // Check for alliance
        if (!auto.startsWith("Red_") && !auto.startsWith("Blu_")) {
            auto = (isRedAlliance ? "Red" : "Blu") + "_" + auto;
        }

        // Check for VSLAM connection
        if(!isVSLAMConnected) {
            auto =  auto + AutoConstants.kNoVSLAMPostfix;
        }
        
        if (autoPaths.keySet().contains(auto)) { // If the auto exists as a red/blue auto already, don't flip it
            flipForRed = false;
        } else if (isRedAlliance && auto.startsWith("Red_")) { // If red auto doesn't exist, use blue auto and flip it
            auto = auto.replace("Red_", "Blu_");
            assert autoPaths.keySet().contains(auto) : "ERROR: you need to create " + auto;
            flipForRed = true;
        } else { // Auto doesn't exist, use default auto
            System.out
                .println("ERROR: no such auto path name found in src/main/deploy/pathplanner/autos: " + auto + 
                    ", switching to default auto " + AutoConstants.kAutoDefault);
            auto = "Blu_" + AutoConstants.kAutoDefault + (!isVSLAMConnected? AutoConstants.kNoVSLAMPostfix : "");
            flipForRed = isRedAlliance;
        }

        selectedAutoName = auto;
        selectedPPAuto = new PathPlannerAuto(selectedAutoName);

        // View choice on smartdashboard
        SmartDashboard.putString("SelectedAuto", selectedAutoName);
        SmartDashboard.putBoolean("FlipForRed", flipForRed);
        SmartDashboard.putBoolean("IsRedAlliance", isRedAlliance);

        Logger.recordOutput("PathPlannerStartPose", new PathPlannerAuto(selectedAutoName).getStartingPose());
    }
}