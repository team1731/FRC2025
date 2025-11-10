package frc.lib.frc1731.subsystem;

import frc.lib.frc1731.hardware.motor.MotorIO;

public abstract class CoupledMotorVelocitySubsystem<M extends MotorIO> extends SingleMotorVelocitySubsystem<M> {
    protected M followerMotor;

    public CoupledMotorVelocitySubsystem(M ioFollower, boolean enabled) {
        super(enabled);
        this.followerMotor = ioFollower;
        this.followerMotor.follow(leadMotor, leadMotor.isInverted() != ioFollower.isInverted());
    }
}