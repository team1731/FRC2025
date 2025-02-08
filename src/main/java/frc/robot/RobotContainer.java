// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// (SCH) This RobotContainer can just be replaced with the old one.
// However, look in the configureBindings() method for another (SCH) note.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ScoreCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.state.score.GamePiece;
import frc.robot.state.score.ScoreStateMachine;
import frc.robot.subsystems.*;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.hand.HandIntakeSubsystem;
import frc.robot.subsystems.hand.HandClamperSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

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
  private ArmSubsystem armSubsystem;
  private HandClamperSubsystem handClamperSubsystem;
  private HandIntakeSubsystem handIntakeSubsystem;
  private ScoreStateMachine scoreStateMachine;

  public RobotContainer(
      CommandSwerveDrivetrain s_driveSubsystem,
      VisionSubsystem s_visionSubsystem,
      LEDStringSubsystem s_ledstring,
      ElevatorSubsystem s_elevatorSubsystem,
      ArmSubsystem s_ArmSubsystem,
      HandClamperSubsystem s_HandClamperSubsystem,
      HandIntakeSubsystem s_HandIntakeSubsystem) {

    driveSubsystem = s_driveSubsystem;
    elevatorSubsystem = s_elevatorSubsystem;
    visionSubsystem = s_visionSubsystem;
    m_ledstring = s_ledstring;
    armSubsystem = s_ArmSubsystem;
    handClamperSubsystem = s_HandClamperSubsystem;
    handIntakeSubsystem = s_HandIntakeSubsystem;

    scoreStateMachine = new ScoreStateMachine(elevatorSubsystem, armSubsystem, handClamperSubsystem, handIntakeSubsystem);

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

 //  xboxController.a().whileTrue(drivetrain.applyRequest(() -> brake));
   // xboxController.b().whileTrue(drivetrain.applyRequest(
     //   () -> point.withModuleDirection(new Rotation2d(-xboxController.getLeftY(), -xboxController.getLeftX()))));

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

    
    // Intake coral
    ky.whileTrue(new IntakeCommand(handClamperSubsystem, handIntakeSubsystem, GamePiece.CORAL));
    ka.whileTrue(new IntakeCommand(handClamperSubsystem, handIntakeSubsystem, GamePiece.ALGAE));
    kx.onTrue(new SequentialCommandGroup(
      new InstantCommand(() -> handClamperSubsystem.close()),
      new InstantCommand(() -> handIntakeSubsystem.stop())
    ));

    // score coral
    kb.whileTrue(new ScoreCommand(scoreStateMachine, elevatorSubsystem, armSubsystem));


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
}
