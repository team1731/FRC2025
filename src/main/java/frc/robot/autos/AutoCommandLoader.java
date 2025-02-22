package frc.robot.autos;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoFireResetSequenceCommand;
import frc.robot.commands.AutoFireSequenceCommand;
import frc.robot.commands.ResetSequenceCommand;
import frc.robot.state.sequencer.Action;
import frc.robot.state.sequencer.GamePiece;
import frc.robot.state.sequencer.Level;
import frc.robot.state.sequencer.SequenceInput;
import frc.robot.state.sequencer.SequenceManager;
import frc.robot.state.sequencer.SequenceStateMachine;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.hand.HandClamperSubsystem;
import frc.robot.subsystems.hand.HandIntakeSubsystem;

public class AutoCommandLoader {
    private SequenceStateMachine sequenceStateMachine;

    // subsystems
    private ElevatorSubsystem elevatorSubsystem;
    private ArmSubsystem armSubsystem;
    private HandIntakeSubsystem handIntakeSubsystem;
    private HandClamperSubsystem handClamperSubsystem;

    
    public AutoCommandLoader(ElevatorSubsystem elevatorSubsystem, ArmSubsystem armSubsystem, HandClamperSubsystem handClamperSubsystem, HandIntakeSubsystem handIntakeSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.armSubsystem = armSubsystem;
        this.handClamperSubsystem = handClamperSubsystem;
        this.handIntakeSubsystem = handIntakeSubsystem;
        sequenceStateMachine = SequenceManager.getStateMachine(elevatorSubsystem, armSubsystem, handClamperSubsystem, handIntakeSubsystem);
    }

    public void registerAutoEventCommands() {
        NamedCommands.registerCommand("CoralFeederIntake", getCoralFeederIntakeCommand());
        NamedCommands.registerCommand("FinishCoralFeederIntake", getFinishCoralFeederIntakeCommand());
        NamedCommands.registerCommand("CoralL4Score", getCoralL4ScoreCommand());
        NamedCommands.registerCommand("FinishCoralScore", new InstantCommand(() -> sequenceStateMachine.setInput(SequenceInput.SCORE)));
        NamedCommands.registerCommand("AlgaeReefL2Intake", getAlgaeReefIntakeCommand(Level.L2));
        NamedCommands.registerCommand("AlgaeReefL3Intake", getAlgaeReefIntakeCommand(Level.L3));
        NamedCommands.registerCommand("FinishAlgaeIntake", new InstantCommand(() -> sequenceStateMachine.setInput(SequenceInput.FINISH_INTAKE)));
        NamedCommands.registerCommand("AlgaeBargeScore", getAlgaeBargeScoreCommand());
    }

    public SequentialCommandGroup getCoralFeederIntakeCommand() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> SequenceManager.setGamePieceSelection(GamePiece.CORAL)),
            new InstantCommand(() -> SequenceManager.setLevelSelection(Level.L2)),
            new InstantCommand(() -> SequenceManager.setActionSelection(Action.INTAKE)),
            new ResetSequenceCommand(elevatorSubsystem, armSubsystem, handClamperSubsystem, handIntakeSubsystem),
            new AutoFireSequenceCommand(elevatorSubsystem, armSubsystem, handClamperSubsystem, handIntakeSubsystem)
        );
    }

    public Command getFinishCoralFeederIntakeCommand() {
        // Here we will use auto reset, b/c we want it to fire and forget
        return new AutoFireResetSequenceCommand(elevatorSubsystem, armSubsystem, handClamperSubsystem, handIntakeSubsystem);
    }

    public SequentialCommandGroup getCoralL4ScoreCommand() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> SequenceManager.setGamePieceSelection(GamePiece.CORAL)),
            new InstantCommand(() -> SequenceManager.setLevelSelection(Level.L4)),
            new InstantCommand(() -> SequenceManager.setActionSelection(Action.SCORE)),
            new AutoFireSequenceCommand(elevatorSubsystem, armSubsystem, handClamperSubsystem, handIntakeSubsystem)
        );
    }

    public SequentialCommandGroup getAlgaeReefIntakeCommand(Level level) {
        return new SequentialCommandGroup(
            new InstantCommand(() -> SequenceManager.setGamePieceSelection(GamePiece.ALGAE)),
            new InstantCommand(() -> SequenceManager.setLevelSelection(level)),
            new InstantCommand(() -> SequenceManager.setActionSelection(Action.INTAKE)),
            new ResetSequenceCommand(elevatorSubsystem, armSubsystem, handClamperSubsystem, handIntakeSubsystem),
            new AutoFireSequenceCommand(elevatorSubsystem, armSubsystem, handClamperSubsystem, handIntakeSubsystem)
        );
    }

    public SequentialCommandGroup getAlgaeBargeScoreCommand() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> SequenceManager.setGamePieceSelection(GamePiece.ALGAE)),
            new InstantCommand(() -> SequenceManager.setLevelSelection(Level.L4)),
            new InstantCommand(() -> SequenceManager.setActionSelection(Action.SCORE)),
            new AutoFireSequenceCommand(elevatorSubsystem, armSubsystem, handClamperSubsystem, handIntakeSubsystem)
        );
    }
}
