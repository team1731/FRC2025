package frc.lib.frc1731.hardware.motor.rev;

import frc.lib.frc1731.PIDGains;
import frc.lib.frc1731.Utils;
import frc.lib.frc1731.hardware.motor.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkBase.*;
import com.revrobotics.spark.config.SparkBaseConfig;

/**
 * Wrapper class for rev motors that use a spark motor controller
 */
public abstract class OLD_MotorIOSparkBase<M extends SparkBase, C extends SparkBaseConfig> extends OLD_MotorIO {
    protected M motor;
    protected SparkClosedLoopController motorCtrl;
    protected RelativeEncoder encoder;
    protected C config;

    protected OLD_MotorIOSparkBase(String bus, M motor, C config, boolean inverted) {
        super(new PortConfig(bus, motor.getDeviceId(), inverted));
        this.motor = motor;
        this.motorCtrl = motor.getClosedLoopController();
        this.encoder = motor.getEncoder();
        
        this.config = config;
        this.config.inverted(inverted);
    }

    @Override
    public void configureMotionProfile(double maxVelocity, double maxAcceleration) {
        this.config.closedLoop.maxMotion
            .maxVelocity(maxVelocity, ClosedLoopSlot.kSlot0)
            .maxAcceleration(maxAcceleration, ClosedLoopSlot.kSlot0)
            .positionMode(com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode.kMAXMotionTrapezoidal, ClosedLoopSlot.kSlot0)
            .allowedClosedLoopError(0, ClosedLoopSlot.kSlot0)
        ;

        this.motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void setInverted(boolean inverted) {
        this.config.inverted(inverted);
        this.motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void withGains(PIDGains gains) {
        super.motorPIDGains.add(gains.getSlot(), gains);
        
        ClosedLoopSlot slot;
        if (gains.getSlot() == 1) {
            slot = ClosedLoopSlot.kSlot1;
        } else if (gains.getSlot() == 2) {
            slot = ClosedLoopSlot.kSlot2;
        } else if (gains.getSlot() == 3) {
            slot = ClosedLoopSlot.kSlot3;
        } else {
            slot = ClosedLoopSlot.kSlot0;
        }

        this.config.closedLoop
            .p(gains.kP, slot)
            .i(gains.kI, slot)
            .d(gains.kD, slot)
            .velocityFF(gains.kV, slot)
            ;

        if (gains.softLimit) {
            this.config.softLimit.forwardSoftLimitEnabled(gains.softLimit);
            this.config.softLimit.forwardSoftLimit(gains.softLimitMax);
            this.config.softLimit.reverseSoftLimitEnabled(gains.softLimit);
            this.config.softLimit.reverseSoftLimit(gains.softLimitMax);
        }

        // this.config.closedLoop.maxMotion
        //     .maxVelocity(gains.maxVelocity, slot)
        //     .maxAcceleration(gains.maxAcceleration, slot)
        //     .positionMode(com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode.kMAXMotionTrapezoidal, slot)
        //     .allowedClosedLoopError(gains.tolerance, slot)
        // ;

        this.motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void setPercentOutput(double percent) {
        this.motor.set(percent);
    }

    @Override
    public void setVoltage(double voltage, int slot) {
        this.motor.setVoltage(voltage);
    }

    @Override
    public void setVelocity(double desiredVelocityRPM, int pidSlot) {
        if (motorPIDGains.size() > 0) {
            // Utils.clamp(
            //     desiredVelocityRPM, 
            //     -motorPIDGains.get(pidSlot).maxVelocity,
            //     motorPIDGains.get(pidSlot).maxVelocity
            // );
        }

        ClosedLoopSlot slot = getSlot(pidSlot);

        this.motorCtrl.setReference(
            desiredVelocityRPM, 
            ControlType.kMAXMotionVelocityControl, 
            slot
        );
    }

    @Override
    public void setPosition(double desiredRotations, int pidSlot) {
        if (motorPIDGains != null) {
            Utils.clamp(
                desiredRotations, 
                motorPIDGains.get(pidSlot).softLimitMin,
                motorPIDGains.get(pidSlot).softLimitMax
            );
        }
        
        this.motorCtrl.setReference(desiredRotations, ControlType.kMAXMotionPositionControl, getSlot(pidSlot));
    }

    @Override
    @SuppressWarnings("unchecked")
    public void setFollowerTo(OLD_MotorIO master, boolean reversed) {
        this.config.follow(((OLD_MotorIOSparkBase<M, C>)master).motor);
        this.motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void setCurrentLimit(int currentAmps) {
        this.config.smartCurrentLimit(currentAmps);
        this.config.secondaryCurrentLimit(currentAmps);
        this.motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void setIdleMode(IdleMode idleMode) {
        com.revrobotics.spark.config.SparkBaseConfig.IdleMode mode;
        switch (idleMode) {
            case kCoast:
                mode = com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kCoast;
                break;
            case kBrake:
            // On default set to brake mode
            default:
            mode = com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kBrake;
                break;
        }

        this.config.idleMode(mode);
        this.motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public double getVelocityRPM() {
        return encoder.getVelocity();
    }

    @Override
    public double getRotations() {
        return encoder.getPosition();
    }

    @Override
    public double getAppliedVoltage() {
        return motor.getBusVoltage();
    }

    @Override
    public void resetEncoderPosition(double rotations) {
        this.encoder.setPosition(rotations);
    }

    @Override
    public double getVoltage() {
        return motor.getAppliedOutput();
    }

    @Override
    public void setSoftLimits(double min, double max) {
        this.config.softLimit.forwardSoftLimitEnabled(true);
        this.config.softLimit.forwardSoftLimit(max);

        this.config.softLimit.reverseSoftLimitEnabled(true);
        this.config.softLimit.reverseSoftLimit(min);
    }

    /**
     * Gets the target PID slot in a useable form
     */
    private ClosedLoopSlot getSlot(int slot) {
        switch (slot) {
            case 1:
                return ClosedLoopSlot.kSlot1;
            case 2:
                return ClosedLoopSlot.kSlot2;
            case 3:
                return ClosedLoopSlot.kSlot3;
            default:
                return ClosedLoopSlot.kSlot0;
        }
    }
}