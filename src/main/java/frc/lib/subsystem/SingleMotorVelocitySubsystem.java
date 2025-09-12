package frc.lib.subsystem;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.hardware.motor.MotorIO;

public abstract class SingleMotorVelocitySubsystem<M extends MotorIO> extends BaseSubsystem {
    protected M leadMotor;

    protected double desiredVelocityRPM = 0.0;

    protected SingleMotorVelocitySubsystem(M io, boolean enabled) {
        super(enabled);
        this.leadMotor = io;
    }

    protected void setVelocityRPM(double desiredRPM) {
        this.setVelocityRPM(desiredRPM, 0);
    }

    protected void setVelocityRPM(double desiredRPM, int pidSlot) {
        this.desiredVelocityRPM = desiredRPM;
        this.leadMotor.setVelocity(desiredRPM, pidSlot);
    }

    protected void setPercentOutput(double desiredPercent) {
        this.desiredVelocityRPM = desiredPercent * 6000d; // TODO - Adjust max velocity based on motor
        this.leadMotor.setPercentOutput(desiredPercent);
    }

    protected void setVoltage(double desiredVoltage) {
        this.desiredVelocityRPM = desiredVoltage * 500d; // TODO - Adjust max velocity based on motor
        this.leadMotor.setVoltage(desiredVoltage);
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
}