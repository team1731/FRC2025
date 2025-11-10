package frc.lib.subsystem;

import frc.lib.hardware.motor.MotorIO;

public abstract class CoupledMotorVelocitySubsystem<M extends MotorIO> extends SingleMotorVelocitySubsystem<M> {
    protected M followerMotor;

    public CoupledMotorVelocitySubsystem(M ioFollower, boolean enabled) {
        super(enabled);
        this.followerMotor = ioFollower;
        this.followerMotor.follow(leadMotor, leadMotor.isInverted() != ioFollower.isInverted());
    }
}