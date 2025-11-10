package frc.lib.subsystem;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.hardware.motor.MotorIO;

public abstract class SingleMotorVelocitySubsystem<M extends MotorIO> extends BaseSubsystem {
    protected M leadMotor;

    protected double targetVelocityRPM = 0.0;

    protected SingleMotorVelocitySubsystem(boolean enabled) {
        super(enabled);
        initializeHardware();
    }

    protected abstract void initializeHardware();

    protected void setVelocityRPM(double targetRPM) {
        this.setVelocityRPM(targetRPM, 0);
    }

    protected void setVelocityRPM(double targetRPM, int pidSlot) {
        this.targetVelocityRPM = targetRPM;
        this.leadMotor.setVelocityRPS(targetRPM, pidSlot);
    }

    protected void setPercentOutput(double targetPercent) {
        this.targetVelocityRPM = targetPercent * 6000d; // TODO - Adjust max velocity based on motor
        this.leadMotor.setPercentOutput(targetPercent);
    }

    protected void setVoltage(double targetVoltage) {
        this.targetVelocityRPM = targetVoltage * 500d; // TODO - Adjust max velocity based on motor
        this.leadMotor.setVoltage(targetVoltage);
    }

    protected double getVelocityRPM() {
        return this.leadMotor.getVelocityRPS();
    }

    protected double getTargetVelocityRPM() {
        return this.leadMotor.getRotations();
    }

    protected Command setVelocityCommand(double desiredRPM) {
        return this.run(() -> this.setVelocityRPM(desiredRPM));
    }

    protected Command setPercentOutputCommand(double desiredPercent) {
        return this.run(() -> this.setPercentOutput(desiredPercent));
    }

    protected Command setVoltageCommand(double desiredVoltage) {
        return this.run(() -> this.setVoltage(desiredVoltage));
    }

    protected Command stopCommand() {
        return this.runOnce(() -> this.setVelocityRPM(0.0));
    }
}