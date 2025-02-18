package frc.robot.autos;

import java.util.HashMap;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoFactory {
    private static boolean flipRedBlue;

    public static boolean isFlipRedBlue() {
        return flipRedBlue;
      }
    
    public static Command getAutonomousCommand(String autoName, boolean redAlliance) {
        HashMap<String, String> autoPaths = AutoLoader.getAutoPaths();

        System.out.println("got here 1");

        String alliancePathName = autoName;
        if (!autoName.startsWith("Red_") && !autoName.startsWith("Blu_")) {
        alliancePathName = (redAlliance ? "Red" : "Blu") + "_" + autoName;
        }
        System.out.println("got here 2");
        // if the named auto (red or blue) exists, use it as-is and do NOT flip the
        // field (red/blue)
        if (autoPaths.keySet().contains(alliancePathName)) {
        flipRedBlue = false;
        }
        // if the named auto does not exist (so there isn't a red one), use the blue one
        // and flip the field
        else if (redAlliance && alliancePathName.startsWith("Red_")) {
        alliancePathName = alliancePathName.replace("Red_", "Blu_");
        assert autoPaths.keySet().contains(alliancePathName) : "ERROR: you need to create " + alliancePathName;
        flipRedBlue = true;
        } else {
        System.out
            .println("ERROR: no such auto path name found in src/main/deploy/pathplanner/autos: " + alliancePathName);
        }
        //System.out.println("About to get Auto Path: " + alliancePathName);
System.out.println("got here 2");
        Command command = new PathPlannerAuto(alliancePathName);
        assert command != null : "ERROR: unable to get AUTO path for: " + alliancePathName + ".auto";
        System.out.println("\nAUTO CODE being used by the software --> " + alliancePathName + ", RED/BLUE flipping is "
            + (flipRedBlue ? "ON" : "OFF") + "\n");
        SmartDashboard.putString("AUTO_FILE_IN_USE", alliancePathName);
        SmartDashboard.putBoolean("RED_BLUE_FLIPPING", flipRedBlue);

        return command;
    }
}
