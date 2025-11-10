package frc.robot.autos;

import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.hand.*;

public class OLD_AutoCommandLoader {
    private ElevatorSubsystem elevatorSubsystem;
    private ArmSubsystem armSubsystem;
    private HandIntakeSubsystem handIntakeSubsystem;
    private HandClamperSubsystem handClamperSubsystem;

    
    public OLD_AutoCommandLoader(ElevatorSubsystem elevatorSubsystem, ArmSubsystem armSubsystem, HandClamperSubsystem handClamperSubsystem, HandIntakeSubsystem handIntakeSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.armSubsystem = armSubsystem;
        this.handClamperSubsystem = handClamperSubsystem;
        this.handIntakeSubsystem = handIntakeSubsystem;
    }

    public void registerAutoEventCommands() {
        // NamedCommands.registerCommand("HoldCoral", new InstantCommand(() -> handClamperSubsystem.holdCoral()));
        // NamedCommands.registerCommand("CoralFeederIntake", getCoralFeederIntakeCommand());
        // NamedCommands.registerCommand("FinishCoralFeederIntake", getFinishCoralFeederIntakeCommand());
        // NamedCommands.registerCommand("CoralL4Score", getCoralL4ScoreCommand());
        // NamedCommands.registerCommand("FinishCoralScore", new AutoFinishCoralScoreCommand(sequenceStateMachine));
        // NamedCommands.registerCommand("AlgaeReefL2Intake", getAlgaeReefIntakeCommand(Level.L2));
        // NamedCommands.registerCommand("AlgaeReefL3Intake", getAlgaeReefIntakeCommand(Level.L3));
        // NamedCommands.registerCommand("FinishAlgaeIntake", new InstantCommand(() -> sequenceStateMachine.setInput(SequenceInput.FINISH_INTAKE)));
        // NamedCommands.registerCommand("AlgaeBargeScore", getAlgaeBargeScoreCommand());
    }

    // public SequentialCommandGroup getCoralFeederIntakeCommand() {
    //     return new SequentialCommandGroup(
    //         new InstantCommand(() -> SequenceManager.setGamePieceSelection(GamePiece.CORAL)),
    //         new InstantCommand(() -> SequenceManager.setLevelSelection(Level.L4)),
    //         new InstantCommand(() -> SequenceManager.setActionSelection(Action.INTAKE)),
    //         new ResetSequenceCommand(elevatorSubsystem, armSubsystem, handClamperSubsystem, handIntakeSubsystem),
    //         new AutoFireSequenceCommand(elevatorSubsystem, armSubsystem, handClamperSubsystem, handIntakeSubsystem)
    //     );
    // }

    // public Command getFinishCoralFeederIntakeCommand() {
    //     // Here we will use auto reset, b/c we want it to fire and forget
    //     return new AutoFireResetSequenceCommand(elevatorSubsystem, armSubsystem, handClamperSubsystem, handIntakeSubsystem);
    // }

    // public SequentialCommandGroup getCoralL4ScoreCommand() {
    //     return new SequentialCommandGroup(
    //         new InstantCommand(() -> SequenceManager.setGamePieceSelection(GamePiece.CORAL)),
    //         new InstantCommand(() -> SequenceManager.setLevelSelection(Level.L4)),
    //         new InstantCommand(() -> SequenceManager.setActionSelection(Action.SCORE)),
    //         new AutoFireSequenceCommand(elevatorSubsystem, armSubsystem, handClamperSubsystem, handIntakeSubsystem)
    //     );
    // }

    // public SequentialCommandGroup getAlgaeReefIntakeCommand(Level level) {
    //     return new SequentialCommandGroup(
    //         new InstantCommand(() -> SequenceManager.setGamePieceSelection(GamePiece.ALGAE)),
    //         new InstantCommand(() -> SequenceManager.setLevelSelection(level)),
    //         new InstantCommand(() -> SequenceManager.setActionSelection(Action.INTAKE)),
    //         new ResetSequenceCommand(elevatorSubsystem, armSubsystem, handClamperSubsystem, handIntakeSubsystem),
    //         new AutoFireSequenceCommand(elevatorSubsystem, armSubsystem, handClamperSubsystem, handIntakeSubsystem)
    //     );
    // }

    // public SequentialCommandGroup getAlgaeBargeScoreCommand() {
    //     return new SequentialCommandGroup(
    //         new InstantCommand(() -> SequenceManager.setGamePieceSelection(GamePiece.ALGAE)),
    //         new InstantCommand(() -> SequenceManager.setLevelSelection(Level.L4)),
    //         new InstantCommand(() -> SequenceManager.setActionSelection(Action.SCORE)),
    //         new AutoFireSequenceCommand(elevatorSubsystem, armSubsystem, handClamperSubsystem, handIntakeSubsystem)
    //     );
    // }
}
