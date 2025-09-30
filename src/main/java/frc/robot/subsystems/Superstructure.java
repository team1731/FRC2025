package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.arm.NEW_ArmSubsystem;

public class Superstructure {
    private NEW_ArmSubsystem arm;

    public enum Level {
        L1,
        L2,
        L3,
        L4
    }

    public Superstructure(NEW_ArmSubsystem arm) {
        this.arm = arm;
    }

    public Command goToCoralPositionCommand(Level level) {
        return Commands.none();
    }

    public Command scoreCoralCommand() {
        return Commands.none();
    }

    public Command homeCommand() {
        return Commands.none();
    }

    public Command intakeCoralCommand() {
        return Commands.none();
    }

    public Command intakeAlgaeCommand() {
        return Commands.none();
    }
}