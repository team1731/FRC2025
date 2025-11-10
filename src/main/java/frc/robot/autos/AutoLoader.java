package frc.robot.autos;

import java.io.File;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Optional;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.vision.VSLAMSubsystem;
import frc.robot.Robot;

public class AutoLoader {
    private static SendableChooser<String> autoChooser = new SendableChooser<>();
    private static HashMap<String, String> autoPaths;

    private static boolean isVSLAMConnected = false;
    private static boolean isRedAlliance = false;
    private static boolean flipForRed = false;

    private static String selectedAuto = "";

    private VSLAMSubsystem vslamSubsystem;

    public AutoLoader(VSLAMSubsystem vslamSubsystem) {
        this.vslamSubsystem = vslamSubsystem;

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
        return Commands.none();
    }

    public Command getSelectedAuto() {
        return autoPreloadCommand().andThen(new PathPlannerAuto(selectedAuto));
    }

    public void update() {
        isVSLAMConnected = vslamSubsystem.isConnected();
        isRedAlliance = this.isRedAlliance();

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

        selectedAuto = auto;

        // View choice on smartdashboard
        SmartDashboard.putString("SelectedAuto", selectedAuto);
        SmartDashboard.putBoolean("FlipForRed", flipForRed);
        SmartDashboard.putBoolean("IsRedAlliance", isRedAlliance);
    }

    private boolean isRedAlliance(){
		Optional<Alliance> alliance = DriverStation.getAlliance();
		if (alliance.isPresent()) {
			return alliance.get() == DriverStation.Alliance.Red;
		}
		return false;
	}
}