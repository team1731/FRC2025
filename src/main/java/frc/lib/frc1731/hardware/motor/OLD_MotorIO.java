package frc.lib.frc1731.hardware.motor;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.simulation.*;
import frc.lib.frc1731.PIDGains;

/**
 * Wrapper class for any motor type that provides a common interface for controlling motors
 */
public abstract class OLD_MotorIO {
    protected List<PIDGains> motorPIDGains = new ArrayList<>();
    protected String canBusName = "";
    protected int deviceID = 0;
    protected boolean inverted = false;
    protected double desiredVelocityRPM = 0d;
    protected EncoderSim simEncoder; 
    protected DCMotorSim simMotor;
    
    protected OLD_MotorIO(PortConfig config) {
        this.canBusName = config.kBus;
        this.deviceID = config.kPort;
        this.inverted = config.kInverted;
    }

    /**
     * Brake behavior for motors
     */
    public enum IdleMode {
        kBrake,
        kCoast
    }

    /**
     * Inverts the output direction of the motor
     */
    public abstract void setInverted(boolean inverted);

    /**
     * Adds a PID gains set to the motor
     */
    public abstract void withGains(PIDGains gains);

    /**
     * Adds multiple PID gains to the motor
     */
    public void withGains(PIDGains... gains) {
        for (PIDGains gain : gains) {
            this.withGains(gain);
        }
    }

    /**
     * Adds a PID gains set to the motor at a particular slot
     */
    public void withGains(PIDGains gains, int slot) {
        this.withGains(gains.setSlot(slot));
    }
    
    /**
     * Sets the motor output to a percentage of maximum output
     */
    public abstract void setPercentOutput(double percent);

    /**
     * Sets the motor output in voltage given a particular PID slot
     */
    public abstract void setVoltage(double voltage, int slot);

    /**
     * Sets the motor output in voltage
     */
    public void setVoltage(double voltage) {
        setVoltage(voltage, 0);
    }

    /**
     * Sets the motor velocity in RPM given a particular PID slot
     */
    public abstract void setVelocity(double desiredRPM, int pidSlot);

    /**
     * Sets the motor velocity in RPM
     */
    public void setVelocity(double desiredRPM) {
        setVelocity(desiredRPM, 0);
    }

    /**
     * Sets the motor position in rotations given a particular PID slot
     */
    public abstract void setPosition(double desiredRotations, int pidSlot);

    /**
     * Sets the motor position in rotations
     */
    public void setPosition(double desiredRotations) {
        setPosition(desiredRotations, 0);
    }
    
    /**
     * Adds an inverted follower motor to this current motor
     */
    public abstract void setFollowerTo(OLD_MotorIO master, boolean reversed);
    
    /**
     * Adds a follower motor to this current motor
     */
    public void setFollowerTo(OLD_MotorIO master) {
        setFollowerTo(master, false);
    }

    /**
     * Sets the current limit of the motor in amps
     */
    public abstract void setCurrentLimit(int currentAmps);

    /**
     * Sets the braking behavior for the motor
     */
    public abstract void setIdleMode(IdleMode idleMode);

    /**
     * Get the current velocity of the motor in RPM
     */
    public abstract double getVelocityRPM();

    /**
     * Get the current position of the motor in rotations
     */
    public abstract double getRotations();

    /**
     * Get the voltage applied to the motor
     */
    public abstract double getAppliedVoltage();

    /**
     * Resets the encoder's position to a specific number of rotations
     */
    public abstract void resetEncoderPosition(double rotations);

    /**
     * Sets the soft limits for the motor in rotations
     */
    public abstract void setSoftLimits(double min, double max);

    /**
     * Configures the motion profile for the trapezoidal control of the motor
     */
    public abstract void configureMotionProfile(double maxVelocity, double maxAcceleration);

    /**
     * Gets the current voltage of the motor in volts
     */
    public abstract double getVoltage();

    /**
     * Whether this motor's output is inverted
     */
    public boolean isInverted() {
        return this.inverted;
    }

    /**
     * Gets the CAN port number of the motor
     */
    public int getDeviceID() {
        return this.deviceID;
    }

    /**
     * Gets the desired velocity in RPM
     */
    public double getDesiredVelocity() {
        return this.desiredVelocityRPM;
    }
}