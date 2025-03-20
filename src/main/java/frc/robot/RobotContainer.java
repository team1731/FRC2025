// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// (SCH) This RobotContainer can just be replaced with the old one.
// However, look in the configureBindings() method for another (SCH) note.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.JoystickConstants;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.ResetSequenceCommand;
import frc.robot.commands.RunSequenceCommand;
import frc.robot.commands.DriveCommand.DriveMode;
import frc.robot.generated.TunerConstants;
import frc.robot.state.sequencer.Action;
import frc.robot.state.sequencer.GamePiece;
import frc.robot.state.sequencer.Level;
import frc.robot.state.sequencer.SequenceManager;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.climb.ClimbConstants;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.hand.HandIntakeSubsystem;
import frc.robot.subsystems.leds.LEDSubsystem;
import frc.robot.subsystems.vision.ReefTarget;
import frc.robot.subsystems.vision.helpers.AprilTagTargetTracker;
import frc.robot.subsystems.hand.HandClamperSubsystem;

public class RobotContainer {
  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
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
  private final Trigger dLeftStick = xboxController.leftStick();
  private final Trigger dLeftBumper = xboxController.leftBumper();
  private final Trigger dRightBumper = xboxController.rightBumper();
  private final Trigger dLeftTrigger = xboxController.leftTrigger();
  private final Trigger dRightTrigger = xboxController.rightTrigger();
  private final Trigger dPOVUp = xboxController.povUp();
  private final Trigger dPOVDown = xboxController.povDown();

  /* Operator Buttons */
  private final Joystick opJoystick1 = new Joystick(1);
  private final JoystickButton opPostA = new JoystickButton(opJoystick1, JoystickConstants.opA);
  private final JoystickButton opPostB = new JoystickButton(opJoystick1, JoystickConstants.opB);
  private final JoystickButton opPostC = new JoystickButton(opJoystick1, JoystickConstants.opC);
  private final JoystickButton opPostD = new JoystickButton(opJoystick1, JoystickConstants.opD);
  private final JoystickButton opPostE = new JoystickButton(opJoystick1, JoystickConstants.opE);
  private final JoystickButton opPostF = new JoystickButton(opJoystick1, JoystickConstants.opF);
  private final JoystickButton opPostG = new JoystickButton(opJoystick1, JoystickConstants.opG);
  private final JoystickButton opPostH = new JoystickButton(opJoystick1, JoystickConstants.opH);
  private final JoystickButton opPostI = new JoystickButton(opJoystick1, JoystickConstants.opI);
  private final JoystickButton opPostJ = new JoystickButton(opJoystick1, JoystickConstants.opJ);
  private final JoystickButton opPostK = new JoystickButton(opJoystick1, JoystickConstants.opK);
  private final JoystickButton opPostL = new JoystickButton(opJoystick1, JoystickConstants.opL);
  private final Joystick opJoystick2 = new Joystick(2);
  private final JoystickButton opbButton1 = new JoystickButton(opJoystick2, JoystickConstants.op1);
  private final JoystickButton opbButton2 = new JoystickButton(opJoystick2, JoystickConstants.op2);
  private final JoystickButton opbButton3 = new JoystickButton(opJoystick2, JoystickConstants.op3);
  private final JoystickButton opbButton4 = new JoystickButton(opJoystick2, JoystickConstants.op4);
  private final JoystickButton opbButton5 = new JoystickButton(opJoystick2, JoystickConstants.op5);
  private final JoystickButton opbButton6 = new JoystickButton(opJoystick2, JoystickConstants.op6);
  private final JoystickButton opbButton7 = new JoystickButton(opJoystick2, JoystickConstants.op7);
  private final JoystickButton opbButton8 = new JoystickButton(opJoystick2, JoystickConstants.op8);
  
  private final Trigger opStart = xboxOperatorController.start();
  private final Trigger opBack = xboxOperatorController.back();
  private final Trigger opY = xboxOperatorController.y();
  private final Trigger opB = xboxOperatorController.b();
  private final Trigger opA = xboxOperatorController.a();
  private final Trigger opX = xboxOperatorController.x();
  private final Trigger opRightBumper = xboxOperatorController.rightBumper();
  private final Trigger opLeftTrigger = xboxOperatorController.leftTrigger();
  private final Trigger opRightTrigger = xboxOperatorController.rightTrigger();

  /* Subsystems */
  private CommandSwerveDrivetrain driveSubsystem;
  private LEDSubsystem ledSubsystem;
  private ElevatorSubsystem elevatorSubsystem;
  private ArmSubsystem armSubsystem;
  private HandClamperSubsystem handClamperSubsystem;
  private HandIntakeSubsystem handIntakeSubsystem;
  private ClimbSubsystem climbSubsystem;

  public RobotContainer(
      CommandSwerveDrivetrain s_driveSubsystem,
      LEDSubsystem s_ledstring,
      ElevatorSubsystem s_elevatorSubsystem,
      ArmSubsystem s_ArmSubsystem,
      HandClamperSubsystem s_HandClamperSubsystem,
      HandIntakeSubsystem s_HandIntakeSubsystem,
      ClimbSubsystem s_ClimbSubsystem
    ) {

    driveSubsystem = s_driveSubsystem;
    elevatorSubsystem = s_elevatorSubsystem;
    ledSubsystem = s_ledstring;
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
      new DriveCommand(driveSubsystem, xboxController)
    );

 
    dStart.onTrue(new InstantCommand(() -> {
      System.out.println("resetting position");
    
      Pose2d resetPosition = Robot.isRedAlliance() ? new Pose2d(10.38, 3.01, new Rotation2d(Math.toRadians(0)))
          : new Pose2d(7.168, 5.006, new Rotation2d(Math.toRadians(180)));
      driveSubsystem.resetPose(resetPosition);
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
    dPOVUp.whileTrue(new InstantCommand(() -> climbSubsystem.moveClimb(ClimbConstants.maxClimbPosition))) 
      .onFalse(new InstantCommand(() -> climbSubsystem.stopClimb()));

    // Climb down
    dPOVDown.whileTrue(new InstantCommand(() -> climbSubsystem.moveClimb(ClimbConstants.minClimbPosition))) 
    .onFalse(new InstantCommand(() -> climbSubsystem.stopClimb()));

    // DRIVER - Controls level selection
    dY.whileTrue(new InstantCommand(() -> SequenceManager.setLevelSelection(Level.L4))); //while pressed set to Level 4 

    dB.whileTrue(new InstantCommand(() -> SequenceManager.setLevelSelection(Level.L3))) //while pressed set to Level 3
      .onFalse(new InstantCommand(() -> SequenceManager.resetLevelToL4())); //if not pressed set default to Level 4 
    
    dA.whileTrue(new InstantCommand(() -> SequenceManager.setLevelSelection(Level.L2))) //while pressed set to Level 2
      .onFalse(new InstantCommand(() -> SequenceManager.resetLevelToL4())); //if not pressed set default to Level 4

    dX.whileTrue(new InstantCommand(() -> SequenceManager.setLevelSelection(Level.L1))) //while pressed set to Level 1
      .onFalse(new InstantCommand(() -> SequenceManager.resetLevelToL4())); //if not pressed set defaullt to Level 4 

    dRightBumper.whileTrue(new InstantCommand(() -> DriveCommand.setDriveMode(DriveMode.TARGETING)))
      .onFalse(new InstantCommand(() -> DriveCommand.setDriveMode(DriveMode.DEFAULT)));

    // OPERATOR - Controls level selection
    opY.whileTrue(new InstantCommand(() -> SequenceManager.setLevelSelection(Level.L4))); //while pressed set to Level 4 

    opB.whileTrue(new InstantCommand(() -> SequenceManager.setLevelSelection(Level.L3))) //while pressed set to Level 3
      .onFalse(new InstantCommand(() -> SequenceManager.resetLevelToL4())); //if not pressed set default to Level 4 
    
    opA.whileTrue(new InstantCommand(() -> SequenceManager.setLevelSelection(Level.L2))) //while pressed set to Level 2
      .onFalse(new InstantCommand(() -> SequenceManager.resetLevelToL4())); //if not pressed set default to Level 4

    opX.whileTrue(new InstantCommand(() -> SequenceManager.setLevelSelection(Level.L1))) //while pressed set to Level 1
      .onFalse(new InstantCommand(() -> SequenceManager.resetLevelToL4())); //if not pressed set defaullt to Level 4 

    // While trigger is true set piece to Algae, when it goes back to false set piece back to Coral
    opLeftTrigger.whileTrue(new InstantCommand(() -> SequenceManager.setGamePieceSelection(GamePiece.ALGAE)))
      .onFalse(new InstantCommand(() -> SequenceManager.setGamePieceSelection(GamePiece.CORAL)));

    //bring up the climb in ready position
    opStart.onTrue(new SequentialCommandGroup(
      new InstantCommand(() -> climbSubsystem.setIsClimbing(true)),
      new InstantCommand(() -> climbSubsystem.moveClimb(ClimbConstants.climbReadyPosition)),
      new InstantCommand(() -> armSubsystem.moveArmNormalSpeed(ArmConstants.halfedArmPosition)) 
    ));

    //bring the climber to the stow position
    opBack.onTrue(new InstantCommand(() -> climbSubsystem.moveClimb(ClimbConstants.climbStowPosition)));

    opRightTrigger.whileTrue(new InstantCommand(() -> SequenceManager.setShouldPluckAlgae(true)))
      .onFalse(new InstantCommand(() -> SequenceManager.setShouldPluckAlgae(false)));

    opRightBumper.onTrue(new InstantCommand(() -> SequenceManager.stateMachineHardReset()));

    // Operator drive to target buttons
    opPostA.onTrue(new InstantCommand(() -> AprilTagTargetTracker.setReefTarget(ReefTarget.A)));

    // opPostB.whileTrue(new InstantCommand(() -> AprilTagTargetTracker.setReefTarget(ReefTarget.B)));
    // opPostC.whileTrue(new InstantCommand(() -> AprilTagTargetTracker.setReefTarget(ReefTarget.C)));
    // opPostD.whileTrue(new InstantCommand(() -> AprilTagTargetTracker.setReefTarget(ReefTarget.D)));
    // opPostE.whileTrue(new InstantCommand(() -> AprilTagTargetTracker.setReefTarget(ReefTarget.E)));
    // opPostF.whileTrue(new InstantCommand(() -> AprilTagTargetTracker.setReefTarget(ReefTarget.F)));
    // opPostG.whileTrue(new InstantCommand(() -> AprilTagTargetTracker.setReefTarget(ReefTarget.G)));
    // opPostH.whileTrue(new InstantCommand(() -> AprilTagTargetTracker.setReefTarget(ReefTarget.H)));
    // opPostI.whileTrue(new InstantCommand(() -> AprilTagTargetTracker.setReefTarget(ReefTarget.I)));
    // opPostJ.whileTrue(new InstantCommand(() -> AprilTagTargetTracker.setReefTarget(ReefTarget.J)));
    // opPostK.whileTrue(new InstantCommand(() -> AprilTagTargetTracker.setReefTarget(ReefTarget.K)));
    // opPostL.whileTrue(new InstantCommand(() -> AprilTagTargetTracker.setReefTarget(ReefTarget.L)));

    opPostA.onTrue(new InstantCommand(() -> System.out.println("button A")));
    opPostB.onTrue(new InstantCommand(() -> System.out.println("button B")));
    opPostC.onTrue(new InstantCommand(() -> System.out.println("button C")));
    opPostD.onTrue(new InstantCommand(() -> System.out.println("button D")));
    opPostE.onTrue(new InstantCommand(() -> System.out.println("button E")));
    opPostF.onTrue(new InstantCommand(() -> System.out.println("button F")));
    opPostG.onTrue(new InstantCommand(() -> System.out.println("button G")));
    opPostH.onTrue(new InstantCommand(() -> System.out.println("button H")));
    opPostI.onTrue(new InstantCommand(() -> System.out.println("button I")));
    opPostJ.onTrue(new InstantCommand(() -> System.out.println("button J")));
    opPostK.onTrue(new InstantCommand(() -> System.out.println("button K")));
    opPostL.onTrue(new InstantCommand(() -> System.out.println("button L")));

    opbButton1.onTrue(new InstantCommand(() -> System.out.println("button 1")));
    opbButton2.onTrue(new InstantCommand(() -> System.out.println("button 2")));
    opbButton3.onTrue(new InstantCommand(() -> System.out.println("button 3")));
    opbButton4.onTrue(new InstantCommand(() -> System.out.println("button 4")));
    opbButton5.onTrue(new InstantCommand(() -> System.out.println("button 5")));
    opbButton6.onTrue(new InstantCommand(() -> System.out.println("button 6")));
    opbButton7.onTrue(new InstantCommand(() -> System.out.println("button 7")));
    opbButton8.onTrue(new InstantCommand(() -> System.out.println("button 8")));

    driveSubsystem.registerTelemetry(logger::telemeterize);
  }
}
