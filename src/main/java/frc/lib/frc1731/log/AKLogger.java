package frc.lib.frc1731.log;

import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggedPowerDistribution;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

/**
 * Class to handle AdvantageKit logging to USB and NetworkTables
 */
public class AKLogger {
    public static void start() {
        /*
		 * Note: do not think this is implemented in the gradle build, if we want to
		 * print this we will need to carry that over
		 */
		// String buildBranch = "N/A";
		// String buildCommitHash = "N/A";
		// String buildDate = "N/A";

		// try {
		// 	File buildInfoFile = new File(Filesystem.getDeployDirectory(), "DeployedBranchInfo.txt");
		// 	if (buildInfoFile.exists() && buildInfoFile.canRead()) {
		// 		Scanner reader = new Scanner(buildInfoFile);
		// 		int i = 0;
		// 		while (reader.hasNext()) {
		// 			if (i == 0) { buildBranch = reader.nextLine();
		// 			} else if (i == 1) { buildCommitHash = reader.nextLine();
		// 			} else { buildDate = reader.nextLine(); }

		// 			i++;
		// 		}
		// 		reader.close();
		// 	}
		// } catch (FileNotFoundException fnf) {
		// 	System.err.println("DeployedBranchInfo.txt not found");
		// 	fnf.printStackTrace();
		// }
		// Logger.recordMetadata("Build Info - Branch", buildBranch);
		// Logger.recordMetadata("Build Info - Commit Hash", buildCommitHash);
		// Logger.recordMetadata("Build Info - Date", buildDate);
    
        Logger.recordMetadata("Event", DriverStation.getEventName());
		Logger.recordMetadata("Game", DriverStation.getGameSpecificMessage());
        Logger.recordMetadata("Robot", "MacDyver");
        Logger.recordMetadata("Team", "Team1731");

        DataLogManager.start();
        DriverStation.startDataLog(DataLogManager.getLog());
        Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables

		String time = DateTimeFormatter.ofPattern("yy-MM-dd_HH-mm-ss").format(LocalDateTime.now());
        if (Robot.isReal()) { // Log to USB stick only if we are on an actual robot
            String logPath = "/U/"+ time +".wpilog";
            Logger.addDataReceiver(new WPILOGWriter(logPath)); // Log to a USB stick
            LoggedPowerDistribution.getInstance(1, ModuleType.kRev); // Enables power distribution logging
        }

        Logger.start();
        SmartDashboard.updateValues();
    }
}