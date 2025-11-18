package frc.lib.frc1731.subsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.lib.frc1731.log.SmartLogger;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class BaseSubsystem extends SubsystemBase {
    private boolean enabled = false;
    protected SmartLogger logger;

    protected BaseSubsystem(boolean enabled) {
        this.enabled = enabled;
        this.logger = new SmartLogger(getName());
    }

    /**
     * Whether we want the subsystem's hardware to be commandable
     */
    public boolean isEnabled() {
        return enabled;
    }

    /**
     * Whether there is a command actively running on this subsystem
     */
    public boolean isCurrentlyCommanded(){
        return getCurrentCommand() != null && getCurrentCommand() != getDefaultCommand();
    }

    /**
     * Set the default command of a subsystem (what to run if no other command requiring it is running)
     */
    @Override
    public void setDefaultCommand(Command command) {
        super.setDefaultCommand(command.withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    }

    /**
     * Set the default command of a subsystem (what to run if no other command requiring it is running)
     *
     * @param command to command to set as default
     */
    public void setDefaultCommand(Runnable runnable) {
        this.setDefaultCommand(run(runnable));
    }

    /**
     * Returns the current command that is actively running on this subsystem
     */
    public Command getCurrentCommand() {
        if (super.getCurrentCommand() == null) { // No command currently running
            return Commands.none();
        }
        return super.getCurrentCommand();
    }

    /**
     * Returns the default command for this subsystem
     */
    public Command getDefaultCommand() {
        if (super.getDefaultCommand() == null) { // No default command
            return Commands.none();
        }
        return super.getDefaultCommand();
    }

    @Override
    public Command run(Runnable runnable) {
        return Commands.either(super.run(runnable), Commands.none(), () -> isEnabled());
    }

    @Override
    public Command runOnce(Runnable runnable) {
        return Commands.either(super.runOnce(runnable), Commands.none(), () -> isEnabled());
    }

    @Override
    public Command runEnd(Runnable runnable, Runnable end) {
        return Commands.either(super.runEnd(runnable, end), Commands.none(), () -> isEnabled());
    }

    public abstract void periodicTelemetry(); // Periodic logging

    public void periodicOutput() {} // If needed, periodic output to hardware

    @Override
    public void periodic () {
        periodicOutput();
        periodicTelemetry();
        logger.log("Command/Actively Commanded", isCurrentlyCommanded());
        logger.log("Command/Has Default Command", !getDefaultCommand().equals(Commands.none()));
        logger.log("Command/Active Command", getCurrentCommand().getName());
        logger.log("Command/Default Command", getDefaultCommand().getName());
    }
}