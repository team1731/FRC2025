package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.state.score.Action;
import frc.robot.state.score.GamePiece;
import frc.robot.state.score.ScoreInput;
import frc.robot.state.score.ScoreStateMachine;
import frc.robot.state.score.sequence.SequenceFactory;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

public class ScoreCommand extends Command {
    ScoreStateMachine m_scoreStateMachine;
    ElevatorSubsystem m_elevatorSubsystem;
    ArmSubsystem m_armSubsystem;
    boolean m_sequenceStarted = false;

    public ScoreCommand(ScoreStateMachine stateMachine, ElevatorSubsystem elevatorSubsystem, ArmSubsystem armSubsystem) {
        m_scoreStateMachine = stateMachine;
        m_elevatorSubsystem = elevatorSubsystem;
        m_armSubsystem = armSubsystem;
        addRequirements(m_elevatorSubsystem, m_armSubsystem);

        // TODO move to button bindings in RobotContainer to allow operator selection
        SequenceFactory.setOperatorActionSelection(Action.SCORE_BARGE);
        SequenceFactory.setOperatorPieceSelection(GamePiece.ALGAE);
    }

    public void runStateMachine() {
        System.out.println("ScoreCommand: running score command state machine");
        m_sequenceStarted = true;
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
