// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.Optional;
import java.util.OptionalInt;
import java.util.Scanner;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.AutoConstants;
import frc.robot.autos.AutoCommandLoader;
import frc.robot.autos.AutoFactory;
import frc.robot.autos.AutoLoader;
import frc.robot.generated.TunerConstants;
import frc.robot.state.sequencer.GamePiece;
import frc.robot.state.sequencer.Level;
import frc.robot.state.sequencer.SequenceManager;
import frc.robot.util.log.MessageLog;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.hand.HandIntakeSubsystem;
import frc.robot.subsystems.leds.LEDSubsystem;
import frc.robot.subsystems.hand.HandClamperSubsystem;
import frc.robot.subsystems.vision.AprilTagSubsystem;
import frc.robot.subsystems.vision.VSLAMSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
	private PathPlannerAuto m_autonomousCommand;
	private SendableChooser<String> autoChooser;
	private AutoCommandLoader autoCommandLoader;
	private String autoCode;
	private String currentKeypadCommand = "";
	private boolean redAlliance = false;
	private int stationNumber = 0;
	public static long millis = System.currentTimeMillis();

	private CommandSwerveDrivetrain driveSubsystem;
	private ElevatorSubsystem elevatorSubsystem;
	private ArmSubsystem armSubsystem;
	private HandClamperSubsystem handClamperSubsystem;
	private HandIntakeSubsystem handIntakeSubsystem;
	private ClimbSubsystem climbSubsystem;
	private boolean lastVSLAMConnectedCheck;
	private Pose2d currentPose;
	private Pose2d targetPose;
	private double autoStartTime;

	public Robot() {
	}

	// SUBSYSTEM DECLARATION
	private LEDSubsystem ledSubsystem;

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
		DataLogManager.start();
		//LogWriter.setupLogging();
		MessageLog.start();
		System.out.println("\n\n\n>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>  EVENT: " + DriverStation.getEventName()
				+ " <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<\n\n");
		AprilTagSubsystem.setupPortForwarding();
		LiveWindow.disableAllTelemetry();

		/*
		 * Instantiate subsystems and provide them to the robot container
		 */
		driveSubsystem = new CommandSwerveDrivetrain(true, TunerConstants.DrivetrainConstants, TunerConstants.FrontLeft,
				TunerConstants.FrontRight, TunerConstants.BackLeft, TunerConstants.BackRight);

		ledSubsystem = new LEDSubsystem(true);

		elevatorSubsystem = new ElevatorSubsystem(true);

		armSubsystem = new ArmSubsystem(true);

		handClamperSubsystem = new HandClamperSubsystem(true);

		handIntakeSubsystem = new HandIntakeSubsystem(true);

		climbSubsystem = new ClimbSubsystem(true);
		armSubsystem.setClimbSubsystem(climbSubsystem);

		// Instantiate our robot container. This will perform all of our button bindings,
		new RobotContainer(driveSubsystem, ledSubsystem, elevatorSubsystem, armSubsystem, handClamperSubsystem, handIntakeSubsystem, climbSubsystem);
		
		/*
		 * Complete initialization setup/configuration
		 */
		initSubsystems();
		System.out.println("creating state machine in robot");
		autoChooser = AutoLoader.loadAutoChooser();
		autoCommandLoader = new AutoCommandLoader(elevatorSubsystem, armSubsystem, handClamperSubsystem, handIntakeSubsystem);
		autoCommandLoader.registerAutoEventCommands();
		autoPreload();
		setupSmartDashboard();
		PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
			currentPose = pose;
		});

		PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
			targetPose = pose;
		});
		FollowPathCommand.warmupCommand().schedule();
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
	private void autoPreload() {
		//m_autonomousCommand = null;
		if(autoChooser == null) return;

		/*
		 * Check for conditions that could require a change to the auto command
		 * 1. Different auto selected by the drive team
		 * 2. Alliance changed
		 * 3. VSLAM connection status changed
		 */
		boolean autoCodeChanged = false;
		String selectedAutoCode = autoChooser.getSelected();
		if(selectedAutoCode == null) {
			selectedAutoCode = autoCode == null ? Constants.AutoConstants.kAutoDefault : autoCode;
		}
		if(!selectedAutoCode.equals(autoCode)) {
			System.out.println("New Auto Code read from dashboard. OLD: " + autoCode + ", NEW: " + selectedAutoCode);
			System.out.println("\nPreloading AUTO CODE --> " + selectedAutoCode);
			autoCodeChanged = true;
		}

		boolean allianceChanged = false;
		boolean isRedAlliance = Robot.isRedAlliance();
		if(redAlliance != isRedAlliance) {
			System.out.println("\n\n===============>>>>>>>>>>>>>>  WE ARE " + (isRedAlliance ? "RED" : "BLUE")
					+ " ALLIANCE  <<<<<<<<<<<<=========================");
			redAlliance = isRedAlliance;
		//	driveSubsystem.configureInitialPosition();
			allianceChanged = true;
		}
		
		boolean vslamConnectionStatusChanged = false;
		boolean isVSLAMConnected = (driveSubsystem.getVSLAMSubsytem() != null)? driveSubsystem.getVSLAMSubsytem().isConnected() : false; 
		if(isVSLAMConnected != lastVSLAMConnectedCheck) {
			System.out.println("VSLAM connection status changed. VSLAM connection status: " + (isVSLAMConnected? "Connected" : "Disconnected"));
			vslamConnectionStatusChanged = true;
			lastVSLAMConnectedCheck = isVSLAMConnected;
		}

		/*
		 * If any of these above conditions changed, kick off creation of a new auto command
		 */
		if(autoCodeChanged || allianceChanged || vslamConnectionStatusChanged) {
			m_autonomousCommand = null;
			m_autonomousCommand = (PathPlannerAuto) AutoFactory.getAutonomousCommand(selectedAutoCode, redAlliance, isVSLAMConnected);		
			
			if (m_autonomousCommand.getStartingPose() != null) {
			Pose2d startingPose = isRedAlliance? new Pose2d(17.55 - m_autonomousCommand.getStartingPose().getX(), 8.05 - m_autonomousCommand.getStartingPose().getY(),m_autonomousCommand.getStartingPose().getRotation().rotateBy(Rotation2d.k180deg)): m_autonomousCommand.getStartingPose();
            driveSubsystem.resetPose(startingPose);
			}

			if (m_autonomousCommand != null){
				autoCode = selectedAutoCode;
				System.out.println("\n\n=====>>>>>>>>>> PRELOADED AUTONOMOUS COMMAND: " + m_autonomousCommand + "<<<<<<<<<<<<=====/n/n");
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
		driveSubsystem.configureAutoBindings();
		driveSubsystem.configureInitialPosition();
		AprilTagFields.kDefaultField.loadAprilTagLayoutField(); 
		AprilTagSubsystem aprilTagSubsystem = driveSubsystem.getAprilTagSubsystem();
		aprilTagSubsystem.setLEDSubsystem(ledSubsystem);
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

		VSLAMSubsystem vslamSubsystem = driveSubsystem.getVSLAMSubsytem();
		if(vslamSubsystem != null) {
            SmartDashboard.putBoolean("VSLAM Connected", vslamSubsystem.isConnected());
			SmartDashboard.putBoolean("VSLAM Tracking", vslamSubsystem.isTracking());
        }
	}

	/** This function is called once each time the robot enters Disabled mode. */
//   ▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄
//   ██ ▄▄▀█▄ ▄██ ▄▄▄ █ ▄▄▀██ ▄▄▀██ █████ ▄▄▄██ ▄▄▀███▄ ▄██ ▀██ █▄ ▄█▄▄ ▄▄
//   ██ ██ ██ ███▄▄▄▀▀█ ▀▀ ██ ▄▄▀██ █████ ▄▄▄██ ██ ████ ███ █ █ ██ ████ ██
//   ██ ▀▀ █▀ ▀██ ▀▀▀ █ ██ ██ ▀▀ ██ ▀▀ ██ ▀▀▀██ ▀▀ ███▀ ▀██ ██▄ █▀ ▀███ ██
//   ▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀
	@Override
	public void disabledInit() {
		driveSubsystem.getAprilTagSubsystem().startAutoLineup();
	}


//   ▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄
//   ██ ▄▄▀█▄ ▄██ ▄▄▄ █ ▄▄▀██ ▄▄▀██ █████ ▄▄▄██ ▄▄▀████ ▄▄ ██ ▄▄▄██ ▄▄▀█▄ ▄██ ▄▄▄ ██ ▄▄▀█▄ ▄██ ▄▄▀
//   ██ ██ ██ ███▄▄▄▀▀█ ▀▀ ██ ▄▄▀██ █████ ▄▄▄██ ██ ████ ▀▀ ██ ▄▄▄██ ▀▀▄██ ███ ███ ██ ██ ██ ███ ███
//   ██ ▀▀ █▀ ▀██ ▀▀▀ █ ██ ██ ▀▀ ██ ▀▀ ██ ▀▀▀██ ▀▀ ████ █████ ▀▀▀██ ██ █▀ ▀██ ▀▀▀ ██ ▀▀ █▀ ▀██ ▀▀▄
//   ▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀
	@Override
	public void disabledPeriodic() {
		VSLAMSubsystem vslamSubsystem = driveSubsystem.getVSLAMSubsytem();
		if(vslamSubsystem != null) {
            SmartDashboard.putBoolean("VSLAM Connected", vslamSubsystem.isConnected());
        }
		
		// call during periodic to detect changes in auto selection
		autoPreload();

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
		System.out.println("AUTO INIT");
		driveSubsystem.getAprilTagSubsystem().stopAutoLineup();

		CommandScheduler.getInstance().cancelAll();
		
		climbSubsystem.stowClimb();
		autoStartTime = Timer.getFPGATimestamp();

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
		//SmartDashboard.putString("Path running", PathPlannerAuto.currentPathName);
		//SmartDashboard.putNumber("current Pose X", currentPose.getX());
		//SmartDashboard.putNumber ("current Pose Y", currentPose.getY());
		//SmartDashboard.putNumber("target pose X",targetPose.getX());
		//SmartDashboard.putNumber("target Pose Y", targetPose.getY());
		//SmartDashboard.putNumber("PP Error", currentPose.getTranslation().getDistance(targetPose.getTranslation()));
		//SmartDashboard.putNumber("AutoRunningTime", Timer.getFPGATimestamp()- autoStartTime);


		if (m_autonomousCommand != null && (Timer.getFPGATimestamp()- autoStartTime) >= 0.25 && (currentPose.getTranslation().getDistance(targetPose.getTranslation()) > 0.6)) {
			System.out.println("distance is" + currentPose.getTranslation().getDistance(targetPose.getTranslation()));
			m_autonomousCommand.cancel();
			System.out.println("Had to Kill the auto because the target pose and current pose were apart by more than a foot");
		}
	}

//   ▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄
//   █▄▄ ▄▄██ ▄▄▄██ █████ ▄▄▄██ ▄▄▄ ██ ▄▄ ███▄ ▄██ ▀██ █▄ ▄█▄▄ ▄▄
//   ███ ████ ▄▄▄██ █████ ▄▄▄██ ███ ██ ▀▀ ████ ███ █ █ ██ ████ ██
//   ███ ████ ▀▀▀██ ▀▀ ██ ▀▀▀██ ▀▀▀ ██ ██████▀ ▀██ ██▄ █▀ ▀███ ██
//   ▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀
	@Override
	public void teleopInit() {
		driveSubsystem.getAprilTagSubsystem().stopAutoLineup();

		// Record both DS control and joystick data in TELEOP
		MessageLog.getLogger();
		System.out.println("TELEOP INIT");
		// resetting to appropriate defaults post auto
		SequenceManager.setLevelSelection(Level.L4);
		SequenceManager.setGamePieceSelection(GamePiece.CORAL);
		// cancel any outstanding auto commands
		CommandScheduler.getInstance().cancelAll();

		if (m_autonomousCommand != null) {
			m_autonomousCommand.cancel();
		}
		currentKeypadCommand = "";
		SmartDashboard.getString("keypadCommand", currentKeypadCommand);
		climbSubsystem.stowClimb();
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
		driveSubsystem.getAprilTagSubsystem().stopAutoLineup();
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
