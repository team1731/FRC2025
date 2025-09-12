package frc.lib.hardware.motor.ctre;


import static edu.wpi.first.units.Units.*;

import frc.lib.*;
import frc.lib.hardware.motor.*;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.signals.*;
import com.ctre.phoenix6.hardware.TalonFX;

/**
 * Wrapper class for motors that use the TalonFX motor controller
 */
public class MotorIOTalonFX extends MotorIO {
    protected TalonFX motor;

    protected TalonFXConfiguration configuration;

    protected PositionVoltage positionOutput = new PositionVoltage(0);
    protected VelocityVoltage velocityOutput = new VelocityVoltage(0);
    protected DutyCycleOut percentOutput = new DutyCycleOut(0);
    protected MotionMagicVoltage motionMagicOutput = new MotionMagicVoltage(0);
    protected VoltageOut voltageOutput = new VoltageOut(0);

    public MotorIOTalonFX(PortConfig config) {
        super(config);
        
        this.motor = new TalonFX(config.kPort);
        this.configuration = new TalonFXConfiguration();
        this.configuration.MotorOutput.Inverted = inverted ? 
            InvertedValue.Clockwise_Positive :
            InvertedValue.CounterClockwise_Positive;

        this.motor.getConfigurator().apply(configuration);

        this.percentOutput.OverrideBrakeDurNeutral = true;
        this.positionOutput.OverrideBrakeDurNeutral = true;
        this.velocityOutput.OverrideBrakeDurNeutral = true;
        this.motionMagicOutput.OverrideBrakeDurNeutral = true;
        this.voltageOutput.OverrideBrakeDurNeutral = true;
    }

    @Override
    public void configureMotionProfile(double maxAcceleration, double cruiseVelocity){
        MotionMagicConfigs motionMagicConfig = configuration.MotionMagic;

        motionMagicConfig.withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(maxAcceleration));
        motionMagicConfig.withMotionMagicCruiseVelocity(RotationsPerSecond.of(cruiseVelocity));
        motionMagicConfig.withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(maxAcceleration*100));
        motor.getConfigurator().apply(configuration);
    }

    @Override
    public void setInverted(boolean inverted) {
        this.configuration.MotorOutput.Inverted = inverted ? 
            InvertedValue.Clockwise_Positive :
            InvertedValue.CounterClockwise_Positive;

        this.motor.getConfigurator().apply(configuration);
    }

    @Override
    public void withGains(PIDProfile gains) {
        super.motorPIDGains.add(gains.getSlot(), gains);
        
        switch (gains.pidSlot) {
            case 0:
                Slot0Configs slot0Config = configuration.Slot0
                    .withKP(gains.kP)
                    .withKI(gains.kI)
                    .withKD(gains.kD)
                    .withKA(gains.kA)
                    .withKV(gains.kV)
                    .withKS(gains.kS);

                this.motor.getConfigurator().apply(slot0Config);
            case 1:
                Slot1Configs slot1Config = configuration.Slot1
                    .withKP(gains.kP)
                    .withKI(gains.kI)
                    .withKD(gains.kD)
                    .withKA(gains.kA)
                    .withKV(gains.kV)
                    .withKS(gains.kS);

                this.motor.getConfigurator().apply(slot1Config);
                break;
            case 2:
                Slot2Configs slot2Config = configuration.Slot2
                    .withKP(gains.kP)
                    .withKI(gains.kI)
                    .withKD(gains.kD)
                    .withKA(gains.kA)
                    .withKV(gains.kV)
                    .withKS(gains.kS);

                this.motor.getConfigurator().apply(slot2Config);
                break;
            default:
                Slot0Configs slotConfig = configuration.Slot0
                    .withKP(gains.kP)
                    .withKI(gains.kI)
                    .withKD(gains.kD)
                    .withKA(gains.kA)
                    .withKV(gains.kV)
                    .withKS(gains.kS);

                this.motor.getConfigurator().apply(slotConfig);
                break;
        }
    }

    @Override
    public void setPercentOutput(double percent) {
        this.motor.setControl(percentOutput.withOutput(percent));
    }

    @Override
    public void setVoltage(double voltage, int slot) {
        this.motor.setVoltage(voltage);
        motor.getMotorVoltage();
    }

    @Override
    public double getVoltage() {
        return motor.getMotorVoltage().getValueAsDouble();
    }

    @Override
    public void setVelocity(double desiredRPM, int pidSlot) {
        double desiredRPS = desiredRPM / 60.0;
        if (motorPIDGains.get(pidSlot) != null) {
            Utils.clamp(
                desiredRPS, 
                -motorPIDGains.get(pidSlot).maxVelocity,
                motorPIDGains.get(pidSlot).maxVelocity
            );
        }
        this.motor.setControl(velocityOutput.withSlot(pidSlot).withVelocity(desiredRPS));
    }

    @Override
    public void setPosition(double desiredRotations, int pidSlot) {
        if (motorPIDGains.get(pidSlot) != null) {
            Utils.clamp(
                desiredRotations,
                motorPIDGains.get(pidSlot).softLimitMin,
                motorPIDGains.get(pidSlot).softLimitMax
            );
        }
        this.motor.setControl(motionMagicOutput.withSlot(pidSlot).withPosition(desiredRotations));
    }

    @Override
    public void setFollowerTo(MotorIO master, boolean reversed) {
        this.motor.setControl(new Follower(((MotorIOTalonFX)master).motor.getDeviceID(), reversed));
    }

    @Override
    public void setCurrentLimit(int currentAmps) {
        CurrentLimitsConfigs currentConfigs = new CurrentLimitsConfigs();
        currentConfigs.StatorCurrentLimit = currentAmps;
        currentConfigs.StatorCurrentLimitEnable = true;

        this.configuration.CurrentLimits = currentConfigs;
        this.motor.getConfigurator().apply(configuration);
    }

    @Override
    public void setSoftLimits(double min, double max) {

        SoftwareLimitSwitchConfigs soft_limit_motor = new SoftwareLimitSwitchConfigs();
        soft_limit_motor.ForwardSoftLimitEnable = true;
        soft_limit_motor.ReverseSoftLimitEnable = true;

        soft_limit_motor.ForwardSoftLimitThreshold = max;
        soft_limit_motor.ReverseSoftLimitThreshold = min;

        this.configuration.withSoftwareLimitSwitch(soft_limit_motor);

        this.motor.getConfigurator().apply(this.configuration);

    }

    @Override
    public void setIdleMode(IdleMode idleMode) {
        NeutralModeValue neutralMode = NeutralModeValue.Brake;
        switch(idleMode) {
            case kCoast:
                neutralMode = NeutralModeValue.Coast;
                break;
            case kBrake: default: // Should default to brake mode
                break;
        }
        this.motor.setNeutralMode(neutralMode);
    }

    @Override
    public double getVelocityRPM() {
        return this.motor.getVelocity().getValueAsDouble() * 60d;
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
    public void resetEncoderPosition(double rotations) {
        this.motor.setPosition(rotations);
    }
}