package frc.robot.commands;

// import frc.robot.subsystems.Swerve;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.ElevatorSubsystem;


public class ElevatorTestCommand extends Command {
    private ElevatorSubsystem m_ElevatorSubsystem;

    public ElevatorTestCommand(ElevatorSubsystem elevatorSubsystem) {
        m_ElevatorSubsystem = elevatorSubsystem;
    }

    @Override
    public void initialize() {
        m_ElevatorSubsystem.moveElevator(15000);
    }
}