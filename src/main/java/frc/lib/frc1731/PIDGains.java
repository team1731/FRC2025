package frc.lib.frc1731;


import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.controller.*;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.util.sendable.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Helper class that is used to store and utilize PID and other hardware constants
 * 
 * - Credit to team 1885 ILITE Robotics for the inital idea
 */
public class PIDGains implements Sendable {
    public int pidSlot = 0;

    public double kP = 0;
    public double kI = 0;
    public double kD = 0;

    public double kIZone = 0;

    public double kS = 0;
    public double kA = 0;
    public double kV = 0;
    public double kG = 0;

    public double tolerance = 0.1; // Set to 0.1 units at default

    public boolean continuousInput = false;
    public double continuousMin = Double.NEGATIVE_INFINITY;
    public double continuousMax = Double.POSITIVE_INFINITY;

    public boolean softLimit = false;
    public double softLimitMin = Double.NEGATIVE_INFINITY;
    public double softLimitMax = Double.POSITIVE_INFINITY;

    public double scale = 1.0;

    public int currentLimit = -1;

    public boolean useSmartMotion = false;

    public SimpleMotorFeedforward feedForward = null;

    private static int instances = 0;

    public PIDGains() {
        SendableRegistry.add(this, this.getClass().getSimpleName(), instances);
        instances++;
    }

    /**
     * Sets the basic P, I, and D gains
     */
    public PIDGains setPID(double p, double i, double d) {
        this.kP = p;
        this.kI = i;
        this.kD = d;
        return this;
    }


    /**
     * Sets the P gain to the desired value
     */
    public PIDGains setP(double gain) {
        kP = gain;
        return this;
    }

    /**
     * Returns P gain
     */
    public double getP() {
        return kP;
    }

    /**
     * Sets the I gain to the desired value along with the I zone
     */
    public PIDGains setI(double gain, double zone) {
        kI = gain;
        kIZone = zone;
        return this;
    }

    /**
     * Sets the I gain to the desired value
     */
    public PIDGains setI(double gain) {
        kI = gain;
        return this;
    }

    /**
     * Sets the I zone to the desired range
     */
    public PIDGains setIZone(double zone) {
        kIZone = zone;
        return this;
    }

    /**
     * Returns I gain
     */
    public double getI() {
        return kI;
    }

    /**
     * Returns I zone
     */
    public double getIZone() {
        return kIZone;
    }

    /**
     * Sets the D gain to the desired value
     */
    public PIDGains setD(double gain) {
        kD = gain;
        return this;
    }

    /**
     * Returns the D gain
     */
    public double getD() {
        return kD;
    }

    /**
     * Sets the V gain to the desired value
     */
    public PIDGains setV(double gain) {
        this.kV = gain;
        this.refreshFF();
        return this;
    }

    /**
     * Returns the V gain
     */
    public double getV() {
        return this.kV;
    }

    /**
     * Sets the A gain to the desired value
     */
    public PIDGains setA(double gain) {
        this.kA = gain;
        this.refreshFF();
        return this;
    }

    /**
     * Returns the A gain
     */
    public double getA() {
        return this.kA;
    }

    /**
     * Sets the S gain to the desired value
     */
    public PIDGains setS(double gain) {
        this.kS = gain;
        this.refreshFF();
        return this;
    }

    /**
     * Returns the S gain
     */
    public double getS() {
        return this.kS;
    }

    /**
     * Sets the G gain to the desired value
     */
    public PIDGains setG(double gain) {
        this.kG = gain;
        return this;
    }

    /**
     * Returns the G gain
     */
    public double getG() {
        return this.kG;
    }

    /**
     * Sets the pid slot to the desired slot
     */
    public PIDGains setSlot(int slot) {
        pidSlot = slot;
        return this;
    }

    /**
     * Returns the pid slot for this PIDProfile
     */
    public int getSlot() {
        return pidSlot;
    }

    /**
     * Sets the position tolerance to the desired range
     */
    public PIDGains setTolerance(double tolerance) {
        this.tolerance = tolerance;
        return this;
    }

    public double getTolerance() {
        return tolerance;
    }

    /**
     * Useful for continually rotating mechanisms
     */
    public PIDGains setContinuousInput(double min, double max) {
        this.continuousMin = min;
        this.continuousMax = max;
        this.continuousInput = true;
        return this;
    }

    public boolean isContinuousInput() {
        return continuousInput;
    }

    /**
     * Currently only works for neo motors
     */
    public PIDGains setSoftLimits(double min, double max) {
        this.softLimitMin = min;
        this.softLimitMax = max;
        this.softLimit = true;
        return this;
    }

    /**
     * Whether this set of pid constants constains a soft limit
     */
    public boolean hasSoftLimits() {
        return softLimit;
    }

    /**
     * Double array returns both min and max -> [minimum input, maximum input]
     */
    public float[] getSoftLimits() {
        return new float[] {(float)softLimitMin, (float)softLimitMax};
    }

    /**
     * Sets a scaling factor for the output
     */
    public PIDGains setScalingFactor(double scale) {
        this.scale = scale;
        return this;
    }

    /**
     * Returns the scaling factor
     */
    public double getScalingFactor() {
        return scale;
    }

    /**
     * Sets a maximum current limit
     */
    public PIDGains setCurrentLimit(int limit) {
        this.currentLimit = limit;
        return this;
    }

    /**
     * Returns the maximum current limit
     */
    public int getCurrentLimit() {
        return currentLimit;
    }

    /**
     * If smart motion and/or smart velocity should be applied
     */
    public boolean isSmartMotion() {
        return useSmartMotion;
    }

    /**
     * Creates a PID controller with the specified constants and configurations
     */
    public ProfiledPIDController toProfiledPIDController(double maxVelocity, double maxAcceleration) {
        ProfiledPIDController pidCtrl = new ProfiledPIDController(kP, kI, kD, new Constraints(maxVelocity * scale, maxAcceleration * scale));
        pidCtrl.setTolerance(tolerance);
        pidCtrl.setIZone(kIZone);
        if (continuousInput) {
            pidCtrl.enableContinuousInput(continuousMin, continuousMax);
        }
        return pidCtrl;
    }

    /**
     * Creates a PID controller with the specified constants and configurations
     */
    public PIDController toPIDController() {
        PIDController pidCtrl = new PIDController(kP, kI, kD);
        pidCtrl.setTolerance(tolerance);
        pidCtrl.setIZone(kIZone);
        if (continuousInput) {
            pidCtrl.enableContinuousInput(continuousMin, continuousMax);
        }
        return pidCtrl;
    }

    /**
     * Converts the generic PIDConstants (P, I, D) to a useable form for PathPlanner
     */
    public PIDConstants toPIDConstants() {
        return new PIDConstants(kP, kI, kD);
    }

    /**
     * Duplicates this PIDGains object and returns a new copy
     */
    public PIDGains duplicate() {
        PIDGains copy = new PIDGains();
        copy.setPID(kP, kI, kD);
        copy.kIZone = this.kIZone;
        copy.kS = this.kS;
        copy.kA = this.kA;
        copy.kV = this.kV;
        copy.kG = this.kG;
        copy.tolerance = this.tolerance;
        copy.continuousInput = this.continuousInput;
        copy.continuousMin = this.continuousMin;
        copy.continuousMax = this.continuousMax;
        copy.softLimit = this.softLimit;
        copy.softLimitMin = this.softLimitMin;
        copy.softLimitMax = this.softLimitMax;
        copy.scale = this.scale;
        copy.currentLimit = this.currentLimit;
        copy.useSmartMotion = this.useSmartMotion;
        return copy;
    }

    public PIDGains logOnAdvantageScope() {
        SmartDashboard.putData("PIDGains[" + instances + "]", this);
        return this;
    }

    /**
     * Resets the feedforward object with the updated values for S, V, and A
     */
    private void refreshFF() {
        this.feedForward = new SimpleMotorFeedforward(kS, kV, kA);
    }


    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("PIDGains");
        builder.addDoubleProperty("p", this::getP, this::setP);
        builder.addDoubleProperty("i", this::getI, this::setI);
        builder.addDoubleProperty("d", this::getD, this::setD);
        builder.addDoubleProperty("s", this::getS, this::setS);
        builder.addDoubleProperty("a", this::getA, this::setA);
        builder.addDoubleProperty("v", this::getV, this::setV);
    }
}