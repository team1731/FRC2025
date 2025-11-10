package frc.lib.subsystem;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.Utils;
import frc.lib.hardware.motor.MotorIO;

public abstract class SingleMotorServoSubsystem<M extends MotorIO> extends BaseSubsystem {
    protected M leadMotor = null;

    protected double targetRotations = 0.0;
    protected double epsilon = 0.1; // Tolerance for position control

    public SingleMotorServoSubsystem(boolean enabled, double tolerance) {
        super(enabled);
        this.epsilon = tolerance;
        if (!isEnabled()) return;
        initializeHardware();
    }

    protected abstract void initializeHardware();

    protected void setTolerance(double epsilon) {
        this.epsilon = epsilon;
    }
    
    protected void setRotations(double rotations) {
        this.setRotations(rotations, 0);
    }

    protected void setRotations(double rotations, int pidSlot) {
        this.targetRotations = rotations;
        this.leadMotor.setPosition(rotations, pidSlot);
    }

    protected void setPercentOutput(double desiredPercent) {
        this.leadMotor.setPercentOutput(desiredPercent);
    }

    protected double getRawRotations() {
        return leadMotor.getRotations();
    }

    protected double getTargetRotations() {
        return this.targetRotations;
    }

    protected double getVelocity() {
        return leadMotor.getVelocityRPS();
    }

    protected double getAppliedVoltage() {
        return leadMotor.getAppliedVoltage();
    }

    protected boolean atTargetPosition() {
        return Utils.isWithin(leadMotor.getRotations(), targetRotations, epsilon);
    }

    protected boolean atTargetPosition(double epsilon) {
        return Utils.isWithin(leadMotor.getRotations(), targetRotations, epsilon);
    }

    protected boolean atPosition(double desiredRotations) {
        return Utils.isWithin(leadMotor.getRotations(), desiredRotations, epsilon);
    }

    protected boolean atPosition(double desiredRotations, double epsilon) {
        return Utils.isWithin(leadMotor.getRotations(), desiredRotations, epsilon);
    }

    protected Command setPositionCommand(double position) {
        return this.run(() -> setRotations(position))
        .until(() -> atTargetPosition());
    }
}