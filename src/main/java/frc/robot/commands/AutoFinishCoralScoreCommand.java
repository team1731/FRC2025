package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.state.sequencer.SequenceInput;
import frc.robot.state.sequencer.SequenceState;
import frc.robot.state.sequencer.SequenceStateMachine;

public class AutoFinishCoralScoreCommand extends Command {
    private SequenceStateMachine m_sequenceStateMachine;
    private boolean m_commandDone = false;

    public AutoFinishCoralScoreCommand(SequenceStateMachine stateMachine) {
        m_sequenceStateMachine = stateMachine;
    }

    @Override
    public void initialize() {
        System.out.println("AutoFinishCoralScoreCommand: initializing...");
        m_commandDone = false;
        // if this command is fired then the path is done, we should be where we want to be
        // tell the state machine to score when ready
        m_sequenceStateMachine.setInput(SequenceInput.AUTO_SCORE);
    }

    @Override
    public void execute() {
        SequenceState currentState = (SequenceState)m_sequenceStateMachine.getCurrentState();
        switch(currentState) {
            case LOWERING:
            case FINISHING:
            case HOME:
                // coral has been scored, we're done
                m_commandDone = true;
                break;
            default:
                break;
        }
    }

    @Override
    public void end(boolean interrupted) {
        if(!m_commandDone) {
            System.out.println("AutoFinishCoralScoreCommand: command interrupted");
            m_commandDone = true;
        }
    }

    @Override
    public boolean isFinished() {
        return m_commandDone;
    }
}
