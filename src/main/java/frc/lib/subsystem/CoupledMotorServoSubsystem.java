package frc.lib.subsystem;

import frc.lib.hardware.motor.MotorIO;

public abstract class CoupledMotorServoSubsystem<M extends MotorIO> extends SingleMotorServoSubsystem<M> {
    protected M followerMotor;
    
    public CoupledMotorServoSubsystem(M ioLead, M ioFollower, boolean enabled) {
        super(ioLead, enabled);
        this.followerMotor = ioFollower;
        this.followerMotor.setFollowerTo(ioLead, ioLead.isInverted() != ioFollower.isInverted());
    }
}
