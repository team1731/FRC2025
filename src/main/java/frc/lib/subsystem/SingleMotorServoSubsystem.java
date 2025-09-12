package frc.lib.subsystem;

import frc.lib.Utils;
import frc.lib.hardware.motor.MotorIO;

public abstract class SingleMotorServoSubsystem<M extends MotorIO> extends BaseSubsystem {
    protected M leadMotor;

    protected double desiredRotations = 0.0;
    protected double epsilon = 0.1; // Tolerance for position control

    public SingleMotorServoSubsystem(M io, boolean enabled) {
        super(enabled);
        this.leadMotor = io;
    }
    
    protected void setRotations(double rotations) {
        this.setRotations(rotations, 0);
    }

    protected void setRotations(double rotations, int pidSlot) {
        this.desiredRotations = rotations;
        this.leadMotor.setPosition(rotations, pidSlot);
    }

    protected void setPercentOutput(double desiredPercent) {
        this.leadMotor.setPercentOutput(desiredPercent);
    }

    protected boolean atTargetPosition() {
        return Utils.isWithin(leadMotor.getRotations(), desiredRotations, epsilon);
    }

    protected boolean atTargetPosition(double epsilon) {
        return Utils.isWithin(leadMotor.getRotations(), desiredRotations, epsilon);
    }

    protected boolean atPosition(double desiredRotations) {
        return Utils.isWithin(leadMotor.getRotations(), desiredRotations, epsilon);
    }

    protected boolean atPosition(double desiredRotations, double epsilon) {
        return Utils.isWithin(leadMotor.getRotations(), desiredRotations, epsilon);
    }
}