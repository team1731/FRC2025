package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.state.score.ScoreAction;
import frc.robot.state.score.ScoreInput;
import frc.robot.state.score.ScoreStateMachine;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

public class ScoreCoralCommand extends Command {
    ScoreStateMachine scoreStateMachine;
    ScoreAction scoreAction;
    ElevatorSubsystem m_elevatorSubsystem;
    ArmSubsystem m_armSubsystem;
    boolean sequenceStarted = false;
    //boolean isFinished = false;

    public ScoreCoralCommand(ScoreStateMachine stateMachine, ScoreAction action, ElevatorSubsystem elevatorSubsystem, ArmSubsystem armSubsystem) {
        scoreStateMachine = stateMachine;
        scoreAction = action;
        m_elevatorSubsystem = elevatorSubsystem;
        m_armSubsystem = armSubsystem;
        addRequirements(m_elevatorSubsystem, m_armSubsystem);
    }

    public void runStateMachine() {
        System.out.println("Running state machine!!!!!!!!!!!!!!!");
        sequenceStarted = true;
        scoreStateMachine.setScoreConditions(scoreAction);
        scoreStateMachine.setInput(ScoreInput.BEGIN);
    }

    @Override
    public void initialize() {
        System.out.println("State machine is ready? " + scoreStateMachine.isReady());
        if(scoreStateMachine.isReady()) {
            runStateMachine();
        }
    }

    @Override
    public void execute() {
        if(!sequenceStarted && scoreStateMachine.isReady()) {
            runStateMachine();
        }
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Score command interrupted");
        scoreStateMachine.endSequence();
        //isFinished = true;
    }
    /* 
    @Override
    public boolean isFinished() {
        return isFinished;
    }
        */
}
