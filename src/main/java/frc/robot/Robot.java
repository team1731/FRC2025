// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.Optional;
import java.util.OptionalInt;
import java.util.Scanner;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.OpConstants;
import frc.robot.Constants.OpConstants.LedOption;
import frc.robot.autos.AutoFactory;
import frc.robot.autos.AutoLoader;
import frc.robot.generated.TunerConstants;
import frc.robot.util.log.LogWriter;
import frc.robot.util.log.MessageLog;
import frc.robot.subsystems.*;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
	private RobotContainer m_robotContainer;
	private Command m_autonomousCommand;
	private SendableChooser<String> autoChooser;
	private String autoCode;
	private String currentKeypadCommand = "";
	private boolean redAlliance;
	private int stationNumber = 0;
	public static long millis = System.currentTimeMillis();

	private CommandSwerveDrivetrain driveSubsystem;
	private VisionSubsystem visionSubsystem;
	private ElevatorSubsystem elevatorSubsystem;

	public Robot() {
	}

	// SUBSYSTEM DECLARATION
	private LEDStringSubsystem ledSubsystem;

	// NOTE: FOR TESTING PURPOSES ONLY!
	// private final Joystick driver = new Joystick(0);
	// private final JoystickButton blinker = null; //new JoystickButton(driver,
	// XboxController.Button.kX.value);

	/**
   * This function is run when the robot is first started up and should be used for any
	 * initialization code.
	 * 
   * NOTE: ASCII ART from https://textfancy.com/text-art/  ("small negative")
	 */
//   ▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄
//   ██ ▄▄▀██ ▄▄▄ ██ ▄▄▀██ ▄▄▄ █▄▄ ▄▄███▄ ▄██ ▀██ █▄ ▄█▄▄ ▄▄
//   ██ ▀▀▄██ ███ ██ ▄▄▀██ ███ ███ ██████ ███ █ █ ██ ████ ██
//   ██ ██ ██ ▀▀▀ ██ ▀▀ ██ ▀▀▀ ███ █████▀ ▀██ ██▄ █▀ ▀███ ██
//   ▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀
	@Override
	public void robotInit() {
		//DataLogManager.start();
		//LogWriter.setupLogging();
		MessageLog.start();
		System.out.println("\n\n\n>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>  EVENT: " + DriverStation.getEventName()
				+ " <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<\n\n");
		VisionSubsystem.setupPortForwarding();
		LiveWindow.disableAllTelemetry();

		/*
		 * Instantiate subsystems and provide them to the robot container
		 */
		driveSubsystem = new CommandSwerveDrivetrain(true, TunerConstants.DrivetrainConstants, TunerConstants.FrontLeft,
				TunerConstants.FrontRight, TunerConstants.BackLeft, TunerConstants.BackRight);

		ledSubsystem = new LEDStringSubsystem(true);

		visionSubsystem = new VisionSubsystem(true, driveSubsystem);

		elevatorSubsystem = new ElevatorSubsystem(true);

		// Instantiate our robot container. This will perform all of our button bindings,
		m_robotContainer = new RobotContainer(driveSubsystem, visionSubsystem, ledSubsystem, elevatorSubsystem);
		
		/*
		 * Complete initialization setup/configuration
		 */
		initSubsystems();
		autoChooser = AutoLoader.loadAutoChooser();
		autoInitPreload();
		setupSmartDashboard();
	}

	private void setupSmartDashboard() {
		SmartDashboard.putData(AutoConstants.kAutoCodeKey, autoChooser);
		SmartDashboard.putString("Build Info - Branch", "N/A");
		SmartDashboard.putString("Build Info - Commit Hash", "N/A");
		SmartDashboard.putString("Build Info - Date", "N/A");

		/*
		 * Note: do not think this is implemented in the gradle build, if we want to
		 * print this we will need to carry that over
		 */
		try {
			File buildInfoFile = new File(Filesystem.getDeployDirectory(), "DeployedBranchInfo.txt");
			if (buildInfoFile.exists() && buildInfoFile.canRead()) {
				Scanner reader = new Scanner(buildInfoFile);
				int i = 0;
				while (reader.hasNext()) {
					if (i == 0) {
						SmartDashboard.putString("Build Info - Branch", reader.nextLine());
					} else if (i == 1) {
						SmartDashboard.putString("Build Info - Commit Hash", reader.nextLine());
					} else {
						SmartDashboard.putString("Build Info - Date", reader.nextLine());
					}
					i++;
				}
				reader.close();
			}
		} catch (FileNotFoundException fnf) {
			System.err.println("DeployedBranchInfo.txt not found");
			fnf.printStackTrace();
		}
		SmartDashboard.updateValues();
	}


//   ▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄
//   █▄ ▄██ ▄▄▄ ████ ▄▄▀██ ▄▄▄██ ▄▄▀███ ▄▄▀██ █████ ████▄ ▄█ ▄▄▀██ ▀██ ██ ▄▄▀██ ▄▄▄
//   ██ ███▄▄▄▀▀████ ▀▀▄██ ▄▄▄██ ██ ███ ▀▀ ██ █████ █████ ██ ▀▀ ██ █ █ ██ █████ ▄▄▄
//   █▀ ▀██ ▀▀▀ ████ ██ ██ ▀▀▀██ ▀▀ ███ ██ ██ ▀▀ ██ ▀▀ █▀ ▀█ ██ ██ ██▄ ██ ▀▀▄██ ▀▀▀
//   ▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀
  	public static boolean isRedAlliance(){
		Optional<Alliance> alliance = DriverStation.getAlliance();
		if (alliance.isPresent()) {
			return alliance.get() == DriverStation.Alliance.Red;
		}
		return false;
	}

// ▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄
// ██ ▄▄ ██ ▄▄▄█▄▄ ▄▄████ ▄▄▄ █▄▄ ▄▄█ ▄▄▀█▄▄ ▄▄█▄ ▄██ ▄▄▄ ██ ▀██ ████ ▀██ ██ ██ ██ ▄▀▄ ██ ▄▄▀██ ▄▄▄██ ▄▄▀
// ██ █▀▀██ ▄▄▄███ ██████▄▄▄▀▀███ ███ ▀▀ ███ ████ ███ ███ ██ █ █ ████ █ █ ██ ██ ██ █ █ ██ ▄▄▀██ ▄▄▄██ ▀▀▄
// ██ ▀▀▄██ ▀▀▀███ ██████ ▀▀▀ ███ ███ ██ ███ ███▀ ▀██ ▀▀▀ ██ ██▄ ████ ██▄ ██▄▀▀▄██ ███ ██ ▀▀ ██ ▀▀▀██ ██ 
// ▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀
	private OptionalInt getStationNumber() {
		return DriverStation.getLocation();
	}


//   ▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄
//   █ ▄▄▀██ ██ █▄▄ ▄▄██ ▄▄▄ ███▄ ▄██ ▀██ █▄ ▄█▄▄ ▄▄████ ▄▄ ██ ▄▄▀██ ▄▄▄██ █████ ▄▄▄ █ ▄▄▀██ ▄▄▀
//   █ ▀▀ ██ ██ ███ ████ ███ ████ ███ █ █ ██ ████ ██████ ▀▀ ██ ▀▀▄██ ▄▄▄██ █████ ███ █ ▀▀ ██ ██ 
//   █ ██ ██▄▀▀▄███ ████ ▀▀▀ ███▀ ▀██ ██▄ █▀ ▀███ ██████ █████ ██ ██ ▀▀▀██ ▀▀ ██ ▀▀▀ █ ██ ██ ▀▀ 
//   ▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀
	private void autoInitPreload() {
		m_autonomousCommand = null;

		String selectedAutoCode = autoChooser.getSelected();
		if(selectedAutoCode == null) {
			selectedAutoCode = (autoCode == null ? Constants.AutoConstants.kAutoDefault : autoCode);
		}
		
		if(!selectedAutoCode.equals(autoCode)) {
			System.out.println("New Auto Code read from dashboard. OLD: " + autoCode + ", NEW: " + selectedAutoCode);
			System.out.println("\nPreloading AUTO CODE --> " + selectedAutoCode);

			m_autonomousCommand = AutoFactory.getAutonomousCommand(selectedAutoCode, redAlliance);
			if (m_autonomousCommand != null){
				autoCode = selectedAutoCode;
				System.out.println("\n=====>>> PRELOADED AUTONOMOUS COMMAND: " + m_autonomousCommand);
			} else {
				System.out.println("\nAUTO CODE " + selectedAutoCode + " IS NOT IMPLEMENTED -- STAYING WITH AUTO CODE " + autoCode);
			}
		}
	}


//   ▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄
//   █▄ ▄██ ▀██ █▄ ▄█▄▄ ▄▄████ ▄▄▄ ██ ██ ██ ▄▄▀██ ▄▄▄ ██ ███ ██ ▄▄▄ █▄▄ ▄▄██ ▄▄▄██ ▄▀▄ ██ ▄▄▄ 
//   ██ ███ █ █ ██ ████ ██████▄▄▄▀▀██ ██ ██ ▄▄▀██▄▄▄▀▀██▄▀▀▀▄██▄▄▄▀▀███ ████ ▄▄▄██ █ █ ██▄▄▄▀▀
//   █▀ ▀██ ██▄ █▀ ▀███ ██████ ▀▀▀ ██▄▀▀▄██ ▀▀ ██ ▀▀▀ ████ ████ ▀▀▀ ███ ████ ▀▀▀██ ███ ██ ▀▀▀ 
//   ▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀
	private void initSubsystems() {
		driveSubsystem.configureInitialPosition();
		driveSubsystem.configureAutoBindings();
		ledSubsystem.init();
		ledSubsystem.setColor(LedOption.INIT);
		visionSubsystem.useVision(false);
	}

	/**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
	 *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
	 * SmartDashboard integrated updating.
	 */
// ▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄
// ██ ▄▄▀██ ▄▄▄ ██ ▄▄▀██ ▄▄▄ █▄▄ ▄▄████ ▄▄ ██ ▄▄▄██ ▄▄▀█▄ ▄██ ▄▄▄ ██ ▄▄▀█▄ ▄██ ▄▄▀
// ██ ▀▀▄██ ███ ██ ▄▄▀██ ███ ███ ██████ ▀▀ ██ ▄▄▄██ ▀▀▄██ ███ ███ ██ ██ ██ ███ ███
// ██ ██ ██ ▀▀▀ ██ ▀▀ ██ ▀▀▀ ███ ██████ █████ ▀▀▀██ ██ █▀ ▀██ ▀▀▀ ██ ▀▀ █▀ ▀██ ▀▀▄
// ▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀
	@Override
	public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
		// block in order for anything in the Command-based framework to work.
		CommandScheduler.getInstance().run();
		// m_robotContainer.displayEncoders();
	}

	/** This function is called once each time the robot enters Disabled mode. */
//   ▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄
//   ██ ▄▄▀█▄ ▄██ ▄▄▄ █ ▄▄▀██ ▄▄▀██ █████ ▄▄▄██ ▄▄▀███▄ ▄██ ▀██ █▄ ▄█▄▄ ▄▄
//   ██ ██ ██ ███▄▄▄▀▀█ ▀▀ ██ ▄▄▀██ █████ ▄▄▄██ ██ ████ ███ █ █ ██ ████ ██
//   ██ ▀▀ █▀ ▀██ ▀▀▀ █ ██ ██ ▀▀ ██ ▀▀ ██ ▀▀▀██ ▀▀ ███▀ ▀██ ██▄ █▀ ▀███ ██
//   ▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀
	@Override
	public void disabledInit() {
		ledSubsystem.setBlink(false);
		ledSubsystem.setColor(OpConstants.LedOption.INIT);
	}


//   ▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄
//   ██ ▄▄▀█▄ ▄██ ▄▄▄ █ ▄▄▀██ ▄▄▀██ █████ ▄▄▄██ ▄▄▀████ ▄▄ ██ ▄▄▄██ ▄▄▀█▄ ▄██ ▄▄▄ ██ ▄▄▀█▄ ▄██ ▄▄▀
//   ██ ██ ██ ███▄▄▄▀▀█ ▀▀ ██ ▄▄▀██ █████ ▄▄▄██ ██ ████ ▀▀ ██ ▄▄▄██ ▀▀▄██ ███ ███ ██ ██ ██ ███ ███
//   ██ ▀▀ █▀ ▀██ ▀▀▀ █ ██ ██ ▀▀ ██ ▀▀ ██ ▀▀▀██ ▀▀ ████ █████ ▀▀▀██ ██ █▀ ▀██ ▀▀▀ ██ ▀▀ █▀ ▀██ ▀▀▄
//   ▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀
	@Override
	public void disabledPeriodic() {
		// call during periodic to detect changes in auto selection
		autoInitPreload();

		boolean redAlliance = Robot.isRedAlliance();
		if (this.redAlliance != redAlliance) {
			this.redAlliance = redAlliance;
			System.out.println("\n\n===============>>>>>>>>>>>>>>  WE ARE " + (redAlliance ? "RED" : "BLUE")
					+ " ALLIANCE  <<<<<<<<<<<<=========================");
			// call if alliance designation switches to ensure the right auto command is created
			autoInitPreload();
		}

		if (Robot.isReal()) {
			try {
				OptionalInt stationNumberInt = getStationNumber();
				if (stationNumberInt.isPresent()) {
					int stationNumber = stationNumberInt.getAsInt();
					if (this.stationNumber != stationNumber) {
						this.stationNumber = stationNumber;
						System.out.println("===============>>>>>>>>>>>>>>  WE ARE STATION NUMBER " + stationNumber
								+ "  <<<<<<<<<<<<=========================\n");
					}
				}
			} catch (Exception e) {
				System.out.println("Exception caught while looking for station number! == " + e);
			}
		}
	}


/** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
//   ▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄
//   █ ▄▄▀██ ██ █▄▄ ▄▄██ ▄▄▄ ██ ▀██ ██ ▄▄▄ ██ ▄▀▄ ██ ▄▄▄ ██ ██ ██ ▄▄▄ ███▄ ▄██ ▀██ █▄ ▄█▄▄ ▄▄
//   █ ▀▀ ██ ██ ███ ████ ███ ██ █ █ ██ ███ ██ █ █ ██ ███ ██ ██ ██▄▄▄▀▀████ ███ █ █ ██ ████ ██
//   █ ██ ██▄▀▀▄███ ████ ▀▀▀ ██ ██▄ ██ ▀▀▀ ██ ███ ██ ▀▀▀ ██▄▀▀▄██ ▀▀▀ ███▀ ▀██ ██▄ █▀ ▀███ ██
//   ▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀
	@Override
	public void autonomousInit() {
		visionSubsystem.useVision(false);
		System.out.println("AUTO INIT");
		CommandScheduler.getInstance().cancelAll();

		if (m_autonomousCommand == null) {
			System.out.println("SOMETHING WENT WRONG - UNABLE TO RUN AUTONOMOUS! CHECK SOFTWARE!");
		} else {
			System.out.println("------------> RUNNING AUTONOMOUS COMMAND: " + m_autonomousCommand + " <----------");
			m_autonomousCommand.schedule();
		}
		System.out.println("autonomousInit: End");
	}


//   ▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄
//   █ ▄▄▀██ ██ █▄▄ ▄▄██ ▄▄▄ ██ ▀██ ██ ▄▄▄ ██ ▄▀▄ ██ ▄▄▄ ██ ██ ██ ▄▄▄ ████ ▄▄ ██ ▄▄▄██ ▄▄▀█▄ ▄██ ▄▄▄ ██ ▄▄▀█▄ ▄██ ▄▄▀
//   █ ▀▀ ██ ██ ███ ████ ███ ██ █ █ ██ ███ ██ █ █ ██ ███ ██ ██ ██▄▄▄▀▀████ ▀▀ ██ ▄▄▄██ ▀▀▄██ ███ ███ ██ ██ ██ ███ ███
//   █ ██ ██▄▀▀▄███ ████ ▀▀▀ ██ ██▄ ██ ▀▀▀ ██ ███ ██ ▀▀▀ ██▄▀▀▄██ ▀▀▀ ████ █████ ▀▀▀██ ██ █▀ ▀██ ▀▀▀ ██ ▀▀ █▀ ▀██ ▀▀▄
//   ▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀
	@Override
	public void autonomousPeriodic() {
		if (doSD()) {
			System.out.println("AUTO PERIODIC");
		}
	}

//   ▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄
//   █▄▄ ▄▄██ ▄▄▄██ █████ ▄▄▄██ ▄▄▄ ██ ▄▄ ███▄ ▄██ ▀██ █▄ ▄█▄▄ ▄▄
//   ███ ████ ▄▄▄██ █████ ▄▄▄██ ███ ██ ▀▀ ████ ███ █ █ ██ ████ ██
//   ███ ████ ▀▀▀██ ▀▀ ██ ▀▀▀██ ▀▀▀ ██ ██████▀ ▀██ ██▄ █▀ ▀███ ██
//   ▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀
	@Override
	public void teleopInit() {

		visionSubsystem.useVision(false);
		ledSubsystem.setColor(LedOption.GREEN);
		//driveSubsystem.seedFieldRelative(new Pose2d(new Translation2d(0,0), new Rotation2d(120)));

		// Record both DS control and joystick data in TELEOP
		MessageLog.getLogger();
		System.out.println("TELEOP INIT");
		CommandScheduler.getInstance().cancelAll();
		initSubsystems();

		if (m_autonomousCommand != null) {
			m_autonomousCommand.cancel();
		}
		currentKeypadCommand = "";
		SmartDashboard.getString("keypadCommand", currentKeypadCommand);
	}

//   ▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄
//   ██ ▄▄▀██ ▄▄▄ ████ ▄▄▄ ██ ▄▄▀
//   ██ ██ ██ ███ ████▄▄▄▀▀██ ██ 
//   ██ ▀▀ ██ ▀▀▀ ████ ▀▀▀ ██ ▀▀ 
//   ▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀
	public static boolean doSD() {
		long now = System.currentTimeMillis();
		if (now - millis > 1000) {
			MessageLog.getLogger().flush();
			millis = now;
			return true;
		}
		return false;
	}

	/** This function is called periodically during operator control. */
//   ▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄
//   █▄▄ ▄▄██ ▄▄▄██ █████ ▄▄▄██ ▄▄▄ ██ ▄▄ ████ ▄▄ ██ ▄▄▄██ ▄▄▀█▄ ▄██ ▄▄▄ ██ ▄▄▀█▄ ▄██ ▄▄▀
//   ███ ████ ▄▄▄██ █████ ▄▄▄██ ███ ██ ▀▀ ████ ▀▀ ██ ▄▄▄██ ▀▀▄██ ███ ███ ██ ██ ██ ███ ███
//   ███ ████ ▀▀▀██ ▀▀ ██ ▀▀▀██ ▀▀▀ ██ ███████ █████ ▀▀▀██ ██ █▀ ▀██ ▀▀▀ ██ ▀▀ █▀ ▀██ ▀▀▄
//   ▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀
	@Override
	public void teleopPeriodic() {

		// if(doSD()){
		// System.out.println("TELEOP PERIODIC");
		// }

	}

//   ▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄
//   █▄▄ ▄▄██ ▄▄▄██ ▄▄▄ █▄▄ ▄▄███▄ ▄██ ▀██ █▄ ▄█▄▄ ▄▄
//   ███ ████ ▄▄▄██▄▄▄▀▀███ ██████ ███ █ █ ██ ████ ██
//   ███ ████ ▀▀▀██ ▀▀▀ ███ █████▀ ▀██ ██▄ █▀ ▀███ ██
//   ▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀
	@Override
	public void testInit() {
		// Cancels all running commands at the start of test mode.
		CommandScheduler.getInstance().cancelAll();
	}

	/** This function is called periodically during test mode. */
//   ▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄
//   █▄▄ ▄▄██ ▄▄▄██ ▄▄▄ █▄▄ ▄▄████ ▄▄ ██ ▄▄▄██ ▄▄▀█▄ ▄██ ▄▄▄ ██ ▄▄▀█▄ ▄██ ▄▄▀
//   ███ ████ ▄▄▄██▄▄▄▀▀███ ██████ ▀▀ ██ ▄▄▄██ ▀▀▄██ ███ ███ ██ ██ ██ ███ ███
//   ███ ████ ▀▀▀██ ▀▀▀ ███ ██████ █████ ▀▀▀██ ██ █▀ ▀██ ▀▀▀ ██ ▀▀ █▀ ▀██ ▀▀▄
//   ▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀
	@Override
	public void testPeriodic() {

	}

}
