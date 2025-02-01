package frc.robot.commands;

// import frc.robot.subsystems.Swerve;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.state.Input;
import frc.robot.state.StateMachineCallback;
import frc.robot.subsystems.score.ElevatorSubsystem;


public class ElevatorTestCommand extends Command {
    private ElevatorSubsystem m_ElevatorSubsystem;
    private boolean finishedRaising = false;

    protected StateMachineCallback inputCallback = (Input input) -> {
        finishedRaising = true;
    };

    public ElevatorTestCommand(ElevatorSubsystem elevatorSubsystem) {
        // subsystem feedback, movement is done
        m_ElevatorSubsystem = elevatorSubsystem;
    }

    @Override
    public void initialize() {
        // move elevator up
        m_ElevatorSubsystem.moveElevator(15000);
    }

    @Override
    public void execute() {
        if(finishedRaising) {
            // move elevator down
            m_ElevatorSubsystem.moveElevator(0);
        }
    }
}