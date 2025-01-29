// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// (SCH) This RobotContainer can just be replaced with the old one.
// However, look in the configureBindings() method for another (SCH) note.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.io.File;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LEDStringSubsystem;
import frc.robot.subsystems.VisionSubsystem;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.commands.*;
import frc.robot.subsystems.*;

public class RobotContainer {
  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final Telemetry logger = new Telemetry(MaxSpeed);

  private final CommandXboxController xboxController = new CommandXboxController(0);
  private final CommandXboxController xboxOperatorController = new CommandXboxController(1);

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  /* Drive Controls */
  // private final int translationAxis = XboxController.Axis.kLeftY.value;
  // private final int strafeAxis = XboxController.Axis.kLeftX.value;
  // private final int rotationAxis = XboxController.Axis.kRightX.value;

  /* Driver Buttons */
  private final Trigger kStart = xboxController.start();
  private final Trigger kBack = xboxController.back();
  private final Trigger ky = xboxController.y();
  private final Trigger kb = xboxController.b();
  private final Trigger ka = xboxController.a();
  private final Trigger kx = xboxController.x();
  private final Trigger kLeftBumper = xboxController.leftBumper();
  private final Trigger kRightBumper = xboxController.rightBumper();
  private final Trigger kLeftTrigger = xboxController.leftTrigger();
  private final Trigger kRightTrigger = xboxController.rightTrigger();
  private final Trigger kPOVUp = xboxController.povUp();

  /* Operator Buttons */

  private final Trigger operatorkStart = xboxOperatorController.start();
  private final Trigger operatorBack = xboxOperatorController.back();
  private final Trigger operatorky = xboxOperatorController.y();
  private final Trigger operatorkb = xboxOperatorController.b();
  private final Trigger operatorka = xboxOperatorController.a();
  private final Trigger operatorkx = xboxOperatorController.x();
  private final Trigger operatorkLeftBumper = xboxOperatorController.leftBumper();
  private final Trigger operatorkRightBumper = xboxOperatorController.rightBumper();
  private final Trigger operatorkLeftTrigger = xboxOperatorController.leftTrigger();
  private final Trigger operatorkRightTrigger = xboxOperatorController.rightTrigger();
  private final Trigger operatorkPOVDown = xboxOperatorController.povDown();
  private final Trigger operatorkPOVUp = xboxOperatorController.povUp();
  private final Trigger operatorkPOVLeft = xboxOperatorController.povLeft();
  private final Trigger operatorkPOVRight = xboxOperatorController.povRight();

  /* Subsystems */
  private CommandSwerveDrivetrain driveSubsystem;
  private VisionSubsystem visionSubsystem;
  private final LEDStringSubsystem m_ledstring;
  private ElevatorSubsystem elevatorSubsystem;
  // private Command Blu_10_Command;
  // private Command Red_10_Command;

  /* Auto Paths */
  private static HashMap<String, String> autoPaths;

  private static boolean flipRedBlue;

  public static boolean isFlipRedBlue() {
    return flipRedBlue;
  }

  public RobotContainer(
      CommandSwerveDrivetrain s_driveSubsystem,
      VisionSubsystem s_visionSubsystem,
      LEDStringSubsystem s_ledstring,
      ElevatorSubsystem s_elevatorSubsystem) {

    driveSubsystem = s_driveSubsystem;
    elevatorSubsystem = s_elevatorSubsystem;
    visionSubsystem = s_visionSubsystem;
    m_ledstring = s_ledstring;

    if (driveSubsystem.isEnabled()) {
      // NamedCommands.registerCommand("RotateLeft", 
     // new SequentialCommandGroup(driveSubsystem.rotateRelative(-45.0) ));
      // NamedCommands.registerCommand("RotateRight", 
     // new SequentialCommandGroup(driveSubsystem.rotateRelative(-45.0) ));

      // NamedCommands.registerCommand("Blu_10_Command", Blu_10_Command);
      // NamedCommands.registerCommand("JustShoot", new SequentialCommandGroup(
      // new IntakeShootStateMachineOneShotCommand(intakeShootStateMachine,
      // ISInput.JUST_SHOOT)));

      // buildAuto10();
      // NamedCommands.registerCommand("Red_10_Command", Red_10_Command);
      // NamedCommands.registerCommand("Blu_10_Command", Blu_10_Command);

    }

    // Configure the button bindings
    configureBindings();
  }

  private void configureBindings() {
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-xboxController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
            .withVelocityY(-xboxController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-xboxController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));

    xboxController.a().whileTrue(drivetrain.applyRequest(() -> brake));
    xboxController.b().whileTrue(drivetrain.applyRequest(
        () -> point.withModuleDirection(new Rotation2d(-xboxController.getLeftY(), -xboxController.getLeftX()))));

    // (SCH) NOTE: These sysId calls aren't strictly necessary,
    // though they seem to be a different method of controlling the drivetrain.
    // I would say copy these and assign them to unused buttons in the new
    // RobotContainer to see what they do

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    xboxController.back().and(xboxController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    xboxController.back().and(xboxController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    xboxController.start().and(xboxController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    xboxController.start().and(xboxController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    // reset the field-centric heading on left bumper press
    xboxController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

    drivetrain.registerTelemetry(logger::telemeterize);

    // ky.whileTrue(driveSubsystem.applyRequest(
    // () -> driveAtSpeaker.withVelocityX(-(Math.abs(xboxController.getLeftY()) *
    // xboxController.getLeftY()) * MaxSpeed)
    // .withVelocityY(-(Math.abs(xboxController.getLeftX()) *
    // xboxController.getLeftX()) *
    // MaxSpeed).withTargetDirection(visionSubsystem.getHeadingToSpeakerInRad())
    //
    // )
    // );

    // kRightBumper.whileTrue(new AmpScoringCommand(intakeSubsystem,
    // elevatorSubsystem, wristSubsystem)));
    // kx.whileTrue(new TrapScoringCommand(intakeSubsystem, elevatorSubsystem,
    // wristSubsystem));

    // Comment out the two lines above and uncomment this to tune shooting angles
    // Also uncomment the call to get the distance to the speaker in the period of
    // vision subsystem (that sends the data to smartdashbord among other things)
    // ka.onTrue(new InstantCommand(() -> wristSubsystem.jogDown()))
    // .onFalse(new InstantCommand(() -> wristSubsystem.stopJog()));

    // kb.onTrue(new InstantCommand(() -> wristSubsystem.jogUp()))
    // .onFalse(new InstantCommand(() -> wristSubsystem.stopJog()));

    kStart.onTrue(driveSubsystem.runOnce(() -> driveSubsystem.seedFieldCentric()));

    kStart.onTrue(new InstantCommand(() -> {
      driveSubsystem.resetPose(new Pose2d(1.47, 5.51, new Rotation2d(0)));
      Rotation2d operatorPerspective = Robot.isRedAlliance() ? new Rotation2d(Math.toRadians(180))
          : new Rotation2d(Math.toRadians(0));
      Pose2d resetPosition = Robot.isRedAlliance() ? new Pose2d(15.03, 5.51, operatorPerspective)
          : new Pose2d(1.47, 5.51, operatorPerspective);
      driveSubsystem.resetPose(resetPosition);
      driveSubsystem.setOperatorPerspectiveForward(operatorPerspective); // Just a Hack
    }));

    kx.onTrue(new ElevatorTestCommand(elevatorSubsystem));

    operatorkLeftBumper.onTrue(new InstantCommand(() -> {

    }));
    operatorkRightBumper.onTrue(new InstantCommand(() -> {

    }));

    operatorkPOVDown.onTrue(new InstantCommand(() -> {

    }));
    operatorkPOVUp.onTrue(new InstantCommand(() -> {

    }));

    operatorkPOVLeft.onTrue(new InstantCommand(() -> {

    }));
    operatorkPOVRight.onTrue(new InstantCommand(() -> {

    }));
    /*
     * operatorkStart
     * .onTrue(new InstantCommand(() -> intakeSubsystem.reverseIntake()))
     * .onFalse(new InstantCommand(() -> intakeSubsystem.stopReverseIntake()));
     */

    // Far Shot
    // operatorky.onTrue(new InstantCommand(() -> wristSubsystem.moveWrist(12))) //
    // this is now over the stage
    // .onFalse(new InstantCommand(() -> wristSubsystem.moveWrist(0)));

    // Safe Shot
    // Line Shot
    // operatorka.onTrue(new InstantCommand(() -> wristSubsystem.moveWrist(15*0.6)))
    // .onFalse(new InstantCommand(() -> wristSubsystem.moveWrist(0)));
    // operatorkRightTrigger.onTrue(new JiggleCommand(intakeShootSubsystem,
    // shooterSubsystem));

    operatorBack.whileTrue(new InstantCommand(() -> visionSubsystem.setConfidence(true)));
    operatorBack.whileFalse(new InstantCommand(() -> visionSubsystem.setConfidence(false)));

    driveSubsystem.registerTelemetry(logger::telemeterize);
  }

  /*
  public static String[] deriveAutoModes() {
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
  */
    //FIXME:Not sure how to remove pathplanner but also keep this code
  // public Command getNamedAutonomousCommand(String autoName, boolean redAlliance) {
  //   String alliancePathName = autoName;
  //   if (!autoName.startsWith("Red_") && !autoName.startsWith("Blu_")) {
  //     alliancePathName = (redAlliance ? "Red" : "Blu") + "_" + autoName;
  //   }
  //   // if the named auto (red or blue) exists, use it as-is and do NOT flip the
  //   // field (red/blue)
  //   if (autoPaths.keySet().contains(alliancePathName)) {
  //     flipRedBlue = false;
  //   }
  //   // if the named auto does not exist (so there isn't a red one), use the blue one
  //   // and flip the field
  //   else if (redAlliance && alliancePathName.startsWith("Red_")) {
  //     alliancePathName = alliancePathName.replace("Red_", "Blu_");
  //     assert autoPaths.keySet().contains(alliancePathName) : "ERROR: you need to create " + alliancePathName;
  //     flipRedBlue = true;
  //   } else {
  //     System.out
  //         .println("ERROR: no such auto path name found in src/main/deploy/pathplanner/autos: " + alliancePathName);
  //   }
  //   // System.out.println("About to get Auto Path: " + alliancePathName);

  //   Command command = driveSubsystem.getAutoPath(alliancePathName);
  //   assert command != null : "ERROR: unable to get AUTO path for: " + alliancePathName + ".auto";
  //   System.out.println("\nAUTO CODE being used by the software --> " + alliancePathName + ", RED/BLUE flipping is "
  //       + (flipRedBlue ? "ON" : "OFF") + "\n");
  //   SmartDashboard.putString("AUTO_FILE_IN_USE", alliancePathName);
  //   SmartDashboard.putBoolean("RED_BLUE_FLIPPING", flipRedBlue);

  //   return command;
  // }
  /*
public  void buildAuto10() {

Blu_10_Command =
        Commands.sequence(
            driveSubsystem.getAutoPath("B_10_Pickup_1"),  // p1
            Commands.either(
                driveSubsystem.getAutoPath("B_10_Score1_Pickup2"), //p2, p3
                driveSubsystem.getAutoPath("B_10_Pickup_2"),  // p4
                intakeSubsystem::hasNote),
            Commands.either(
                driveSubsystem.getAutoPath("B_10_Score2_Pickup3"), //p5, p6
                driveSubsystem.getAutoPath("B_10_Pickup_3"), //p7
                intakeSubsystem::hasNote),
            Commands.either(
                driveSubsystem.getAutoPath("B_10_Score3_Pickup4"), //p8, p9
                driveSubsystem.getAutoPath("B_10_Pickup_4"), //p10
                intakeSubsystem::hasNote),
           Commands.either(
                driveSubsystem.getAutoPath("B_10_Score4_Pickup5"), //p11, p12
                driveSubsystem.getAutoPath("B_10_Pickup_5"), //p13
                intakeSubsystem::hasNote),
           Commands.either(
                driveSubsystem.getAutoPath("B_10_Score_5"), //14
                Commands.none(), 
                intakeSubsystem::hasNote));



Red_10_Command =
        Commands.sequence(
            driveSubsystem.getAutoPath("R_10_Pickup_1"),  // p1
            Commands.either(
                driveSubsystem.getAutoPath("R_10_Score1_Pickup2"), //p2, p3
                driveSubsystem.getAutoPath("R_10_Pickup_2"),  // p4
                intakeSubsystem::hasNote),
            Commands.either(
                driveSubsystem.getAutoPath("R_10_Score2_Pickup3"), //p5, p6
                driveSubsystem.getAutoPath("R_10_Pickup_3"), //p7
                intakeSubsystem::hasNote),
            Commands.either(
                driveSubsystem.getAutoPath("R_10_Score3_Pickup4"), //p8, p9
                driveSubsystem.getAutoPath("R_10_Pickup_4"), //p10
                intakeSubsystem::hasNote),
           Commands.either(
                driveSubsystem.getAutoPath("R_10_Score4_Pickup5"), //p11, p12
                driveSubsystem.getAutoPath("R_10_Pickup_5"), //p13
                intakeSubsystem::hasNote),
           Commands.either(
                driveSubsystem.getAutoPath("R_10_Score_5"), //14
                Commands.none(), 
                intakeSubsystem::hasNote));
                

}
   */
}
