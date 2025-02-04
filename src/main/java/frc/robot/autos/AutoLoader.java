package frc.robot.autos;

import java.io.File;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants;
import frc.robot.Robot;

public class AutoLoader {
    private static final SendableChooser<String> autoChooser = new SendableChooser<>();
    private static HashMap<String, String> autoPaths;

    public static HashMap<String, String> getAutoPaths() {
        return autoPaths;
    }

    public static SendableChooser<String> loadAutoChooser() {
        String[] autoModes = getAutoModes();
		for (String autoMode : autoModes) {
			autoChooser.addOption(autoMode, autoMode);
			System.out.println("Added autoMode '" + autoMode + "' to autoChooser.");
		}
		
		// pre-load the default auto
		autoChooser.setDefaultOption(Constants.AutoConstants.kAutoDefault, Constants.AutoConstants.kAutoDefault);
        
        return autoChooser;
    }
    
    private static String[] getAutoModes() {
        autoPaths = findPaths(new File(Filesystem.getLaunchDirectory(),
            (Robot.isReal() ? "home/lvuser" : "src/main") + "/deploy/pathplanner/autos"));
        List<String> autoModes = new ArrayList<String>();
        for (String key : autoPaths.keySet()) {
        String stripKey = key.toString();
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
}
