package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.JoystickConstants;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.DriveCommand.DriveMode;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.climb.ClimbConstants;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.climb.NEW_ClimbSubsystem;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.elevator.NEW_ElevatorSubsystem;
import frc.robot.subsystems.hand.NEW_HandClamperSubsystem;
import frc.robot.subsystems.hand.NEW_HandIntakeSubsystem;
import frc.robot.subsystems.leds.LEDSubsystem;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.Superstructure.*;
import frc.robot.subsystems.vision.ReefTarget;
import frc.robot.subsystems.vision.helpers.AprilTagTargetTracker;

public class NEW_RobotContainer {
    private Superstructure superstructure;
    private final Telemetry logger = new Telemetry(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond));

    /* Subsystems */
    private CommandSwerveDrivetrain driveSubsystem;
    private LEDSubsystem ledSubsystem;
    private ArmSubsystem arm;
    private NEW_ElevatorSubsystem elevator;
    private NEW_HandClamperSubsystem hand;
    private NEW_HandIntakeSubsystem intake;
    private ClimbSubsystem climb;

    /* Driver Buttons */
    private final CommandXboxController xboxController = new CommandXboxController(0);
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
    private final JoystickButton opElevReset = new JoystickButton(sideButtons, JoystickConstants.op6);
    private final JoystickButton opKnockAlgae = new JoystickButton(sideButtons, JoystickConstants.op7);
    private final JoystickButton opAlgae = new JoystickButton(sideButtons, JoystickConstants.op8);

    public NEW_RobotContainer(boolean enabled) {
        configureSubsystems(enabled);
        configureButtonBindings();
    }

    /**
     * Configure all active subsystems on the robot and set default commands
     */
    public void configureSubsystems(boolean enabled) {
        this.driveSubsystem = new CommandSwerveDrivetrain(enabled, TunerConstants.DrivetrainConstants, TunerConstants.FrontLeft,
				TunerConstants.FrontRight, TunerConstants.BackLeft, TunerConstants.BackRight);

        this.ledSubsystem = new LEDSubsystem(enabled);
        this.arm = new ArmSubsystem(enabled);
        this.elevator = new NEW_ElevatorSubsystem(enabled);
        this.hand = new NEW_HandClamperSubsystem(enabled);
        this.intake = new NEW_HandIntakeSubsystem(enabled);
        this.climb = new ClimbSubsystem(enabled);
        this.superstructure = new Superstructure(arm, elevator, hand, intake, climb);

        // Drivetrain will execute this command periodically 
        // if no other command is active on the drivetrain
        this.driveSubsystem.setDefaultCommand(
            new DriveCommand(driveSubsystem, xboxController, sideButtons)
        );

        // Sends swerve telemetry data onto NetworkTables
        this.driveSubsystem.registerTelemetry(logger::telemeterize);
    }

    /**
     * Configure the button bindings
     */
    public void configureButtonBindings() {
        // Reset robot pose and heading
        dPOVRight.onTrue(new InstantCommand(() -> {
            Pose2d resetPosition = Robot.isRedAlliance() ? new Pose2d(10.38, 3.01, new Rotation2d(Math.toRadians(0)))
                : new Pose2d(7.168, 5.006, new Rotation2d(Math.toRadians(180)));
            driveSubsystem.resetPose(resetPosition);
        }));

        // Set algae mode
        opAlgae.whileTrue(superstructure.setAlgaeModeCommand(true))
            .onFalse(superstructure.setAlgaeModeCommand(false));

        // Level selection
        (dY.or(opL4)).whileTrue(superstructure.setLevelCommand(Level.L4)); //while pressed set to Level 4 

        (dB.or(opL3)).whileTrue(superstructure.setLevelCommand(Level.L3)) //while pressed set to Level 3
        .onFalse(superstructure.setLevelCommand(Level.L4)); //if not pressed set default to Level 4 
        
        (dA.or(opL2)).whileTrue(superstructure.setLevelCommand(Level.L2)) //while pressed set to Level 2
        .onFalse(superstructure.setLevelCommand(Level.L4)); //if not pressed set default to Level 4

        (dX.or(opL1)).whileTrue(superstructure.setLevelCommand(Level.L1)) //while pressed set to Level 1
        .onFalse(superstructure.setLevelCommand(Level.L4)); //if not pressed set defaullt to Level 4 

        // Starts the targetting sequence
        dLeftBumper.whileTrue(new InstantCommand(() -> DriveCommand.setDriveMode(DriveMode.TARGETING)))
        .onFalse(new InstantCommand(() -> DriveCommand.setDriveMode(DriveMode.DEFAULT)));

        // Starts intaking sequence
        dLeftTrigger.whileTrue(superstructure.intakeCommand())
        .onFalse(superstructure.finishIntakeCommand());

        // Starts scoring sequence
        dRightTrigger.whileTrue(superstructure.scoreCommand())
        .onFalse(superstructure.finishScoreCommand());

        // Bring up the climb in ready position
        dStart.onTrue(superstructure.setClimbingCommand());

        // Climb up
        dRightBumper.whileTrue(climb.moveClimbCommand(ClimbConstants.maxClimbPosition))
        .onFalse(climb.stopCommand());

        // Climb down
        dPOVDown.whileTrue(climb.moveClimbCommand(ClimbConstants.minClimbPosition))
        .onFalse(climb.stopCommand());

        // Pluck Algae from the reef
        opKnockAlgae.whileTrue(superstructure.setShouldPluckAlgaeCommand(true))
        .onFalse(superstructure.setShouldPluckAlgaeCommand(false));

        // // Rezero the elevator
        opElevReset.onTrue(
            superstructure.setShouldPreventL4Command(true)
            .andThen(elevator.unjamElevatorCommand())
        ).onFalse(
            superstructure.setShouldPreventL4Command(false)
            .andThen(elevator.stopElevatorCommand())
        );
        
        // opL4RestrictionToggle.onTrue(new InstantCommand(() -> SequenceManager.setShouldPreventL4(false)));

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
    }
}