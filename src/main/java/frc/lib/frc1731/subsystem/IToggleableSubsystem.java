package frc.lib.frc1731.subsystem;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface IToggleableSubsystem extends Subsystem {
	boolean isEnabled();
}