package frc.lib.subsystem;

import frc.lib.hardware.motor.MotorIO;

public abstract class CoupledMotorServoSubsystem<M extends MotorIO> extends SingleMotorServoSubsystem<M> {
    protected M followerMotor;
    
    public CoupledMotorServoSubsystem(M follower, boolean enabled, double tolerance) {
        super(enabled, tolerance);
        this.followerMotor = follower;
        this.followerMotor.follow(leadMotor, follower.isInverted() != leadMotor.isInverted());
    }
}
