package frc.lib.hardware;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.*;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.wpilibj.RobotController;
import frc.lib.PIDGains;
import frc.lib.hardware.motor.MotorIO;
import frc.lib.hardware.motor.PortConfig;
import frc.robot.Robot;

public class MotorIOTalonFX extends MotorIO {
    protected TalonFX motor;
    private TalonFXConfiguration cfg;
    private TalonFXConfigurator configurator;
    private TalonFXSimState simState;

    private DynamicMotionMagicVoltage mmOutput;

    private List<PIDGains> pidGains = new ArrayList<>();

    public MotorIOTalonFX(PortConfig config) {
        super(config);
        this.motor = new TalonFX(config.kPort, config.kBus);
        this.configurator = motor.getConfigurator();

        // Set up configuration
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake; // Brake on default

        cfg.MotorOutput.Inverted = config.kInverted ? // Inverted = ccw
            InvertedValue.CounterClockwise_Positive : 
            InvertedValue.Clockwise_Positive;

        this.simState = motor.getSimState();
        this.simState.setSupplyVoltage(Robot.isReal() ? RobotController.getBatteryVoltage() : 12d);

        applyConfigs();
    }

    @Override
    public void follow(MotorIO master, boolean invertedFromMaster) {
        this.motor.setControl(new Follower(((MotorIOTalonFX)master).motor.getDeviceID(), invertedFromMaster));
    }

    @Override
    public void withMotionProfile(double velocity, double acceleration, double jerk) {
        this.mmOutput = new DynamicMotionMagicVoltage(0d, velocity, acceleration, jerk);

        MotionMagicConfigs mm = cfg.MotionMagic;
        mm.MotionMagicCruiseVelocity = velocity; 
        mm.MotionMagicAcceleration = acceleration; 
        mm.MotionMagicJerk = jerk;

        configurator.apply(mm);
        applyConfigs();
    }

    @Override
    public void withPIDGains(PIDGains gains) {
        this.pidGains.add(gains);
        switch (gains.pidSlot) {
            case 0:
                cfg.Slot0
                .withKP(gains.kP)
                .withKI(gains.kI)
                .withKD(gains.kD)
                .withKA(gains.kA)
                .withKV(gains.kV)
                .withKS(gains.kS)
                .withKG(gains.kG);

                applyConfigs();
            case 1:
                cfg.Slot1
                    .withKP(gains.kP)
                    .withKI(gains.kI)
                    .withKD(gains.kD)
                    .withKA(gains.kA)
                    .withKV(gains.kV)
                    .withKS(gains.kS)
                    .withKG(gains.kG);

                applyConfigs();
                break;
            case 2:
                cfg.Slot2
                    .withKP(gains.kP)
                    .withKI(gains.kI)
                    .withKD(gains.kD)
                    .withKA(gains.kA)
                    .withKV(gains.kV)
                    .withKS(gains.kS)
                    .withKG(gains.kG);

                applyConfigs();
                break;
            default:
                cfg.Slot0
                    .withKP(gains.kP)
                    .withKI(gains.kI)
                    .withKD(gains.kD)
                    .withKA(gains.kA)
                    .withKV(gains.kV)
                    .withKS(gains.kS)
                    .withKG(gains.kG);

                applyConfigs();
                break;
        }
    }

    @Override
    public void setSoftLimits(double min, double max) {
        SoftwareLimitSwitchConfigs softLimitMotor = new SoftwareLimitSwitchConfigs();
        softLimitMotor.ForwardSoftLimitEnable = true;
        softLimitMotor.ReverseSoftLimitEnable = true;

        softLimitMotor.ForwardSoftLimitThreshold = max;
        softLimitMotor.ReverseSoftLimitThreshold = min;

        this.cfg.withSoftwareLimitSwitch(softLimitMotor);
        applyConfigs();
    }

    @Override
    public void setNeutralMode(NeutralModeValue mode) {
        this.cfg.MotorOutput.NeutralMode = mode;
    }

    public void setMotionMagicSpeeds(double velocity, double acceleration) {
        this.mmOutput.Velocity = velocity;
        this.mmOutput.Acceleration = acceleration;
    }

    @Override
    public void setPercentOutput(double percent) {
        this.motor.setControl(new DutyCycleOut(percent));
    }

    @Override
    public void setVelocityRPS(double rps, int pidSlot) {
        this.motor.setControl(new VelocityVoltage(rps).withSlot(pidSlot));
    }

    @Override
    public void setVelocityRPS(double rps) {
        this.setVelocityRPS(rps, 0);
    }

    @Override
    public void setPosition(double rotations, int pidSlot) {
        this.motor.setControl(mmOutput.withSlot(pidSlot).withPosition(rotations));
    }

    @Override
    public void setPosition(double rotations) {
        this.setPosition(rotations, 0);
    }

    @Override
    public void setVoltage(double voltage, int slot) {
        this.motor.setControl(new VoltageOut(voltage));
    }

    @Override
    public double getVelocityRPS() {
        return this.motor.getVelocity().getValueAsDouble();
    }

    @Override
    public double getRotations() {
        return this.motor.getPosition().getValueAsDouble();
    }

    @Override
    public double getAppliedVoltage() {
        return this.motor.getMotorVoltage().getValueAsDouble();
    }

    @Override
    public void brake() {
        this.motor.setControl(new NeutralOut());
    }

    @Override
    public boolean isInverted() {
        return true;
    }

    @Override
    public void resetEncoderPosition(double rotations) {
        this.motor.setPosition(rotations);
    }

    @Override
    public void withStatorCurrentLimit(double amps) {
        CurrentLimitsConfigs clc = cfg.CurrentLimits;

        clc.StatorCurrentLimitEnable = true;
        clc.StatorCurrentLimit = amps;
        applyConfigs();
    }

    public void withFeedbackConfigs(FeedbackConfigs configs) {
        this.configurator.apply(configs);
    }

    public void withMotionMagicConfigs(MotionMagicConfigs configs) {
        this.configurator.apply(configs);
    }

    public TalonFXConfiguration getConfiguration() {
        return this.cfg;
    }

    public void applyConfigs() {
        this.configurator.apply(cfg);
    }
}