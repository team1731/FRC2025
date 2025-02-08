package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.state.score.ScoreInput;
import frc.robot.state.score.ScoreStateMachine;
import frc.robot.state.score.sequence.SequenceFactory;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.hand.HandClamperSubsystem;
import frc.robot.subsystems.hand.HandIntakeSubsystem;

public class RunSequenceCommand extends Command {
    ScoreStateMachine m_scoreStateMachine;
    ElevatorSubsystem m_elevatorSubsystem;
    ArmSubsystem m_armSubsystem;
    HandClamperSubsystem m_clamperSubsystem;
    HandIntakeSubsystem m_intakeSubsystem;
    boolean m_sequenceStarted = false;

    public RunSequenceCommand(ScoreStateMachine stateMachine, ElevatorSubsystem elevatorSubsystem, ArmSubsystem armSubsystem, HandClamperSubsystem clamperSubsystem, HandIntakeSubsystem intakeSubsystem) {
        m_scoreStateMachine = stateMachine;
        m_elevatorSubsystem = elevatorSubsystem;
        m_armSubsystem = armSubsystem;
        m_clamperSubsystem = clamperSubsystem;
        m_intakeSubsystem = intakeSubsystem;
        addRequirements(m_elevatorSubsystem, m_armSubsystem, m_clamperSubsystem, m_intakeSubsystem);
    }

    public void runStateMachine() {
        System.out.println("ScoreCommand: running score command state machine");
        m_sequenceStarted = true;
        m_scoreStateMachine.setGamePiece(SequenceFactory.getOperatorPieceSelection());
        m_scoreStateMachine.setSequence(SequenceFactory.getSequence());
        m_scoreStateMachine.setInput(ScoreInput.BEGIN);
    }

    @Override
    public void initialize() {
        if(m_scoreStateMachine.isReady()) {
            runStateMachine();
        }
    }

    @Override
    public void execute() {
        if(!m_sequenceStarted && m_scoreStateMachine.isReady()) {
            runStateMachine();
        }
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("ScoreCommand: command interrupted");
        m_scoreStateMachine.endSequence();
    }
}
