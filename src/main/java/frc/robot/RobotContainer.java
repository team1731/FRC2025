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
import frc.robot.commands.ClimbReadyCommand;
import frc.robot.commands.ResetSequenceCommand;
import frc.robot.commands.RunSequenceCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.state.sequencer.Action;
import frc.robot.state.sequencer.GamePiece;
import frc.robot.state.sequencer.Level;
import frc.robot.state.sequencer.SequenceManager;
import frc.robot.subsystems.*;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.climb.ClimbConstants;
import frc.robot.subsystems.climb.ClimbSubsystem;
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
      .withDeadband(MaxSpeed * 0.05).withRotationalDeadband(MaxAngularRate * 0.05) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final Telemetry logger = new Telemetry(MaxSpeed);

  private final CommandXboxController xboxController = new CommandXboxController(0);
  private final CommandXboxController xboxOperatorController = new CommandXboxController(1);


  /* Drive Controls */
  // private final int translationAxis = XboxController.Axis.kLeftY.value;
  // private final int strafeAxis = XboxController.Axis.kLeftX.value;
  // private final int rotationAxis = XboxController.Axis.kRightX.value;

  /* Driver Buttons */
  private final Trigger dStart = xboxController.start();
  private final Trigger dBack = xboxController.back();
  private final Trigger dY = xboxController.y();
  private final Trigger dB = xboxController.b();
  private final Trigger dA = xboxController.a();
  private final Trigger dX = xboxController.x();
  private final Trigger dLeftBumper = xboxController.leftBumper();
  private final Trigger dRightBumper = xboxController.rightBumper();
  private final Trigger dLeftTrigger = xboxController.leftTrigger();
  private final Trigger dRightTrigger = xboxController.rightTrigger();
  private final Trigger dPOVUp = xboxController.povUp();

  /* Operator Buttons */

  private final Trigger opStart = xboxOperatorController.start();
  private final Trigger opBack = xboxOperatorController.back();
  private final Trigger opY = xboxOperatorController.y();
  private final Trigger opB = xboxOperatorController.b();
  private final Trigger opA = xboxOperatorController.a();
  private final Trigger opX = xboxOperatorController.x();
  private final Trigger opLeftBumper = xboxOperatorController.leftBumper();
  private final Trigger opRightBumper = xboxOperatorController.rightBumper();
  private final Trigger opLeftTrigger = xboxOperatorController.leftTrigger();
  private final Trigger opRightTrigger = xboxOperatorController.rightTrigger();
  private final Trigger opPOVDown = xboxOperatorController.povDown();
  private final Trigger opPOVUp = xboxOperatorController.povUp();
  private final Trigger opPOVLeft = xboxOperatorController.povLeft();
  private final Trigger opPOVRight = xboxOperatorController.povRight();

  /* Subsystems */
  private CommandSwerveDrivetrain driveSubsystem;
  private VisionSubsystem visionSubsystem;
  private final LEDStringSubsystem m_ledstring;
  private ElevatorSubsystem elevatorSubsystem;
  private ArmSubsystem armSubsystem;
  private HandClamperSubsystem handClamperSubsystem;
  private HandIntakeSubsystem handIntakeSubsystem;
  private ClimbSubsystem climbSubsystem;

  public RobotContainer(
      CommandSwerveDrivetrain s_driveSubsystem,
      VisionSubsystem s_visionSubsystem,
      LEDStringSubsystem s_ledstring,
      ElevatorSubsystem s_elevatorSubsystem,
      ArmSubsystem s_ArmSubsystem,
      HandClamperSubsystem s_HandClamperSubsystem,
      HandIntakeSubsystem s_HandIntakeSubsystem,
      ClimbSubsystem s_ClimbSubsystem
      ) {

    driveSubsystem = s_driveSubsystem;
    elevatorSubsystem = s_elevatorSubsystem;
    visionSubsystem = s_visionSubsystem;
    m_ledstring = s_ledstring;
    armSubsystem = s_ArmSubsystem;
    handClamperSubsystem = s_HandClamperSubsystem;
    handIntakeSubsystem = s_HandIntakeSubsystem;
    climbSubsystem = s_ClimbSubsystem;

    // Configure the button bindings
    configureBindings();
  }

  private void configureBindings() {
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.

        driveSubsystem.setDefaultCommand( // Drivetrain will execute this command periodically
        driveSubsystem.applyRequest(
            () -> drive.withVelocityX(-(Math.abs(xboxController.getLeftY()) * xboxController.getLeftY()) * MaxSpeed)                                                                                                                     
                .withVelocityY(-(Math.abs(xboxController.getLeftX()) * xboxController.getLeftX()) * MaxSpeed) 
                .withRotationalRate(-xboxController.getRightX() * MaxAngularRate)
        )
    );




    // (SCH) TODO: These sysId calls aren't strictly necessary,
    // though they seem to be a different method of controlling the drivetrain.
    // I would say copy these and assign them to unused buttons in the new
    // RobotContainer to see what they do

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
   // xboxController.back().and(xboxController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
  //  xboxController.back().and(xboxController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
  //  xboxController.start().and(xboxController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    //xboxController.start().and(xboxController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    // reset the field-centric heading on left bumper press
  //  xboxController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric())); //TODO (SF) we need to reassign this binding

    //drivetrain.registerTelemetry(logger::telemeterize);

  //  dStart.onTrue(driveSubsystem.runOnce(() -> driveSubsystem.seedFieldCentric()));

 
  dB.onTrue(new InstantCommand(() -> {
      System.out.println("resetting position");
    
      Pose2d resetPosition = Robot.isRedAlliance() ? new Pose2d(7.168, 5.006, new Rotation2d(Math.toRadians(0)))
          : new Pose2d(7.168, 5.006, new Rotation2d(Math.toRadians(180)));
      driveSubsystem.resetPose(resetPosition);
    //  driveSubsystem.setOperatorPerspectiveForward(operatorPerspective); // Just a Hack
    }));


    // Sets arm to intake
    dLeftTrigger.whileTrue(new SequentialCommandGroup(
      new InstantCommand(() -> SequenceManager.setActionSelection(Action.INTAKE)),
      new ResetSequenceCommand(elevatorSubsystem, armSubsystem, handClamperSubsystem, handIntakeSubsystem),
      new RunSequenceCommand(elevatorSubsystem, armSubsystem, handClamperSubsystem, handIntakeSubsystem)));

    // Sets arm to score
    dRightTrigger.whileTrue(new SequentialCommandGroup(
      new InstantCommand(() -> SequenceManager.setActionSelection(Action.SCORE)),
      new RunSequenceCommand(elevatorSubsystem, armSubsystem, handClamperSubsystem, handIntakeSubsystem)));

    // Climb up
    dLeftBumper.whileTrue(new InstantCommand(() -> climbSubsystem.moveClimb(ClimbConstants.maxClimbPosition))) 
      .onFalse(new InstantCommand(() -> climbSubsystem.stopClimb()));

    // Climb down
    dRightBumper.whileTrue(new InstantCommand(() -> climbSubsystem.moveClimb(ClimbConstants.minClimbPosition))) 
    .onFalse(new InstantCommand(() -> climbSubsystem.stopClimb()));

    // Controls level selection
    opY.whileTrue(new InstantCommand(() -> SequenceManager.setLevelSelection(Level.L4))) //while pressed set to Level 4
      .onFalse(new InstantCommand(() -> SequenceManager.resetLevelToL2())); //if not pressed set default to Level 2  

    opB.whileTrue(new InstantCommand(() -> SequenceManager.setLevelSelection(Level.L3))) //while pressed set to Level 3
      .onFalse(new InstantCommand(() -> SequenceManager.resetLevelToL2())); //if not pressed set default to Level 2 
    
    opA.whileTrue(new InstantCommand(() -> SequenceManager.setLevelSelection(Level.L2))); //while pressed set to Level 2

    opX.whileTrue(new InstantCommand(() -> SequenceManager.setLevelSelection(Level.L1))) //while pressed set to Level 1
      .onFalse(new InstantCommand(() -> SequenceManager.resetLevelToL2())); //if not pressed set defaullt to Level 2 

    // While trigger is true set piece to Algae, when it goes back to false set piece back to Coral
    opLeftTrigger.whileTrue(new InstantCommand(() -> SequenceManager.setGamePieceSelection(GamePiece.ALGAE)))
      .onFalse(new InstantCommand(() -> SequenceManager.setGamePieceSelection(GamePiece.CORAL)));

    //bring up the climb in ready position
    opStart.onTrue(new SequentialCommandGroup(
      new InstantCommand(() -> climbSubsystem.moveClimb(ClimbConstants.climbReadyPosition)),
      new InstantCommand(() -> armSubsystem.moveArmNormalSpeed(ArmConstants.stowArmPosition)) 
    ));

     //opStart.onTrue(new ResetHandCommand(handClamperSubsystem, handIntakeSubsystem)); //TODO: (SF) can we add this to the SequentialCommandGroup
    
    opBack.whileTrue(new InstantCommand(() -> visionSubsystem.setConfidence(true)))
      .onFalse(new InstantCommand(() -> visionSubsystem.setConfidence(false)));

    driveSubsystem.registerTelemetry(logger::telemeterize);


  }
}
