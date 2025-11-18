package frc.lib.frc1731.hardware.motor;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.lib.frc1731.PIDGains;

public abstract class MotorIO {
    protected List<PIDGains> pidGains = new ArrayList<>();
    protected CANcoder cancoder = null; // Null on default
    protected PortConfig portConfig = null;

    protected MotorIO(PortConfig config) {}

    public abstract void follow(MotorIO master, boolean invertedFromMaster);

    public abstract void withFollower(MotorIO follower, boolean invertedFromMaster);

    public abstract void withMotionProfile(double velocity, double acceleration, double jerk);

    public void withCANCoder(int deviceID, String bus, CANcoderConfiguration configuration) {
        this.cancoder = new CANcoder(deviceID, bus);
        this.cancoder.getConfigurator().apply(configuration);
    }

    public abstract void withPIDGains(PIDGains gains);

    public abstract void setSoftLimits(double min, double max);

    public abstract void setNeutralMode(NeutralModeValue mode);

    public abstract void setPercentOutput(double percent);

    public abstract void setVelocityRPS(double rps, int pidSlot);

    public void setVelocityRPS(double rps) {
        this.setVelocityRPS(rps, 0);
    }

    public abstract void setPosition(double rotations, int pidSlot);

    public void setPosition(double rotations) {
        this.setPosition(rotations, 0);
    }

    public abstract void setVoltage(double voltage, int slot);

    public void setVoltage(double voltage) {
        this.setVoltage(voltage, 0);
    }

    public abstract double getVelocityRPS();

    public abstract double getRotations();

    public abstract double getAppliedVoltage();

    public abstract void brake();

    public abstract void withStatorCurrentLimit(double amps);

    /**
     * Whether this motor's output is inverted
     */
    public boolean isInverted() {
        return this.portConfig.kInverted;
    }

    public abstract void resetEncoderPosition(double position);
}