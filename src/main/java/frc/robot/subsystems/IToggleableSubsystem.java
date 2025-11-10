package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface IToggleableSubsystem extends Subsystem {
	boolean isEnabled();
}
