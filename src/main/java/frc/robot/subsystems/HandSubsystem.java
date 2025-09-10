package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;

public class HandSubsystem extends BaseSubsystem {
    public HandSubsystem(boolean enabled) {
        super(enabled);
    }

    @Override
    public void periodicTelemetry() {
        logger.log("Current Position", 0.0);
        logger.log("Desired Position", 0.0);
        logger.log("At Position", false);
    }
    
    public Command setIntakingCommand() {
        return this.run(() -> {});
    }
}