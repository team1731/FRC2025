package frc.lib.subsystem;

import frc.lib.hardware.motor.MotorIO;

public abstract class CoupledMotorVelocitySubsystem<M extends MotorIO> extends SingleMotorVelocitySubsystem<M> {
    protected M followerMotor;

    public CoupledMotorVelocitySubsystem(M ioLead, M ioFollower, boolean enabled) {
        super(ioLead, enabled);
        this.followerMotor = ioFollower;
        this.followerMotor.setFollowerTo(ioLead, ioLead.isInverted() != ioFollower.isInverted());
    }
}