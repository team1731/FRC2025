// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
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
import frc.robot.state.StateMachine;
import frc.robot.state.sequencer.Action;
import frc.robot.state.sequencer.GamePiece;
import frc.robot.state.sequencer.Level;
import frc.robot.state.sequencer.SequenceManager;
import frc.robot.state.sequencer.SequenceState;
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
  private final Trigger dPOVUpLeft = xboxController.povUpLeft();
  private final Trigger dPOVUp = xboxController.povUp();
  private final Trigger dPOVDown = xboxController.povDown();
  private final Trigger dPOVLeft = xboxController.povLeft();
  private final Trigger dPOVRight = xboxController.povRight();

  /* Operator Buttons */
  private final GenericHID centerButtons = new Joystick(1);
  private final JoystickButton opPostA = new JoystickButton(centerButtons, JoystickConstants.opA);
  private final JoystickButton opPostB = new JoystickButton(centerButtons, JoystickConstants.opB);
  private final JoystickButton opPostC = new JoystickButton(centerButtons, JoystickConstants.opC);
  private final JoystickButton opPostD = new JoystickButton(centerButtons, JoystickConstants.opD);
  private final JoystickButton opPostE = new JoystickButton(centerButtons, JoystickConstants.opE);
  private final JoystickButton opPostF = new JoystickButton(centerButtons, JoystickConstants.opF);
  private final JoystickButton opPostG = new JoystickButton(centerButtons, JoystickConstants.opG);
  private final JoystickButton opPostH = new JoystickButton(centerButtons, JoystickConstants.opH);
  private final JoystickButton opPostI = new JoystickButton(centerButtons, JoystickConstants.opI);
  private final JoystickButton opPostJ = new JoystickButton(centerButtons, JoystickConstants.opJ);
  private final JoystickButton opPostK = new JoystickButton(centerButtons, JoystickConstants.opK);
  private final JoystickButton opPostL = new JoystickButton(centerButtons, JoystickConstants.opL);
  private final GenericHID sideButtons = new Joystick(2);
  private final JoystickButton opL1 = new JoystickButton(sideButtons, JoystickConstants.op1);
  private final JoystickButton opL2 = new JoystickButton(sideButtons, JoystickConstants.op2);
  private final JoystickButton opL3 = new JoystickButton(sideButtons, JoystickConstants.op3);
  private final JoystickButton opL4 = new JoystickButton(sideButtons, JoystickConstants.op4);
  private final JoystickButton opL4RestrictionToggle = new JoystickButton(sideButtons, JoystickConstants.op5);
  //private final JoystickButton opTargetModeToggle = new JoystickButton(sideButtons, JoystickConstants.op5);
  private final JoystickButton opElevReset = new JoystickButton(sideButtons, JoystickConstants.op6);
  private final JoystickButton opKnockAlgae = new JoystickButton(sideButtons, JoystickConstants.op7);
  private final JoystickButton opAlgae = new JoystickButton(sideButtons, JoystickConstants.op8);

  /* Subsystems */
  private CommandSwerveDrivetrain driveSubsystem;
  private LEDSubsystem ledSubsystem;
  private ElevatorSubsystem elevatorSubsystem;
  private ArmSubsystem armSubsystem;
  private HandClamperSubsystem handClamperSubsystem;
  private HandIntakeSubsystem handIntakeSubsystem;
  private ClimbSubsystem climbSubsystem;
  private StateMachine stateMachine;

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
      new DriveCommand(driveSubsystem, xboxController, sideButtons)
    );

 
    dPOVRight.onTrue(new InstantCommand(() -> {
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
    dRightBumper.whileTrue(new InstantCommand(() -> climbSubsystem.moveClimb(ClimbConstants.maxClimbPosition))) 
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

    dLeftBumper.whileTrue(new InstantCommand(() -> DriveCommand.setDriveMode(DriveMode.TARGETING)))
      .onFalse(new InstantCommand(() -> DriveCommand.setDriveMode(DriveMode.DEFAULT)));

    // OPERATOR - Controls level selection
    opL4.whileTrue(new InstantCommand(() -> SequenceManager.setLevelSelection(Level.L4))); //while pressed set to Level 4 

    opL3.whileTrue(new InstantCommand(() -> SequenceManager.setLevelSelection(Level.L3))) //while pressed set to Level 3
      .onFalse(new InstantCommand(() -> SequenceManager.resetLevelToL4())); //if not pressed set default to Level 4 
    
    opL2.whileTrue(new InstantCommand(() -> SequenceManager.setLevelSelection(Level.L2))) //while pressed set to Level 2
      .onFalse(new InstantCommand(() -> SequenceManager.resetLevelToL4())); //if not pressed set default to Level 4

    opL1.whileTrue(new InstantCommand(() -> SequenceManager.setLevelSelection(Level.L1))) //while pressed set to Level 1
      .onFalse(new InstantCommand(() -> SequenceManager.resetLevelToL4())); //if not pressed set defaullt to Level 4 

    // While trigger is true set piece to Algae, when it goes back to false set piece back to Coral
    opAlgae.whileTrue(new InstantCommand(() -> SequenceManager.setGamePieceSelection(GamePiece.ALGAE)))
      .onFalse(new InstantCommand(() -> SequenceManager.setGamePieceSelection(GamePiece.CORAL)));

    //bring up the climb in ready position
dStart.onTrue(
  new ConditionalCommand(
    new SequentialCommandGroup(
      new InstantCommand(() -> climbSubsystem.setIsClimbing(true)),
      new InstantCommand(() -> climbSubsystem.moveClimb(ClimbConstants.climbReadyPosition)),
      new InstantCommand(() -> armSubsystem.moveArmNormalSpeed(ArmConstants.halfedArmPosition))
    ),
    new InstantCommand(() -> {}), // Do nothing if condition is false
    () -> SequenceState.HOME == stateMachine.getCurrentState()
  )
);

     //bring the climber to the stow position 
    // opRightBumper.onTrue(new InstantCommand(() -> climbSubsystem.moveClimb(ClimbConstants.climbStowPosition)));

    opKnockAlgae.whileTrue(new InstantCommand(() -> SequenceManager.setShouldPluckAlgae(true)))
      .onFalse(new InstantCommand(() -> SequenceManager.setShouldPluckAlgae(false)));

    dPOVLeft.onTrue(new InstantCommand(() -> SequenceManager.stateMachineHardReset())); 

    // Rezero the elevator
    opElevReset.onTrue(new SequentialCommandGroup(
      new InstantCommand(() -> SequenceManager.setShouldPreventL4(true)),
      new InstantCommand(() -> elevatorSubsystem.setElevatorUnstuck(true))))
      .onFalse(new SequentialCommandGroup(
        new InstantCommand(() -> elevatorSubsystem.setElevatorUnstuck(false)),
        new InstantCommand(() -> elevatorSubsystem.stopElevator()),
        new InstantCommand(() -> System.out.println("Reset elevator postion"))));
    
    opL4RestrictionToggle.onTrue(new InstantCommand(() -> SequenceManager.setShouldPreventL4(false)));

    // Operator drive to target buttons
    opPostA.whileTrue(new InstantCommand(() -> AprilTagTargetTracker.setReefTarget(ReefTarget.A)));
    opPostB.whileTrue(new InstantCommand(() -> AprilTagTargetTracker.setReefTarget(ReefTarget.B)));
    opPostC.whileTrue(new InstantCommand(() -> AprilTagTargetTracker.setReefTarget(ReefTarget.C)));
    opPostD.whileTrue(new InstantCommand(() -> AprilTagTargetTracker.setReefTarget(ReefTarget.D)));
    opPostE.whileTrue(new InstantCommand(() -> AprilTagTargetTracker.setReefTarget(ReefTarget.E)));
    opPostF.whileTrue(new InstantCommand(() -> AprilTagTargetTracker.setReefTarget(ReefTarget.F)));
    opPostG.whileTrue(new InstantCommand(() -> AprilTagTargetTracker.setReefTarget(ReefTarget.G)));
    opPostH.whileTrue(new InstantCommand(() -> AprilTagTargetTracker.setReefTarget(ReefTarget.H)));
    opPostI.whileTrue(new InstantCommand(() -> AprilTagTargetTracker.setReefTarget(ReefTarget.I)));
    opPostJ.whileTrue(new InstantCommand(() -> AprilTagTargetTracker.setReefTarget(ReefTarget.J)));
    opPostK.whileTrue(new InstantCommand(() -> AprilTagTargetTracker.setReefTarget(ReefTarget.K)));
    opPostL.whileTrue(new InstantCommand(() -> AprilTagTargetTracker.setReefTarget(ReefTarget.L)));

    driveSubsystem.registerTelemetry(logger::telemeterize);
  }
}
