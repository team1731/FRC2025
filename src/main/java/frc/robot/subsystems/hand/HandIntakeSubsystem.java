package frc.robot.subsystems.hand;

import frc.lib.frc1731.hardware.MotorIOTalonFX;
import frc.lib.frc1731.subsystem.VelocitySubsystem;
import edu.wpi.first.wpilibj2.command.Command;

import static frc.robot.subsystems.hand.HandConstants.*;

import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;

public class HandIntakeSubsystem extends VelocitySubsystem<MotorIOTalonFX> {
    public HandIntakeSubsystem(boolean enabled) {
        super(enabled);
        super.withSimulation(intakeSimConstants, intakeGains.logOnAdvantageScope());

    }

    @Override
    protected void initializeHardware() {
        this.motor = new MotorIOTalonFX(intakePortConfig);
        this.motor.withPIDGains(intakeGains);
        this.motor.withVoltageConfigs(intakeVoltageConfigs);
        this.motor.withHardwareLimitSwitchConfigs(intakeLimitSwitchConfigs);
        this.motor.withStatorCurrentLimit(intakeStatorCurrentLimit);
        this.motor.setNeutralMode(NeutralModeValue.Brake);
        this.motor.applyConfigs();
    }

    @Override
    public void periodicTelemetry() {
        logger.log("Current Velocity RPS", getVelocityRPS());
        logger.log("Target Velocity RPS", getTargetVelocityRPS());
    }

    public boolean hasPiece() {
        return motor.getReverseLimit().getValue() == ReverseLimitValue.ClosedToGround;
    }

    public boolean alignedToPole() {
        return motor.getForwardLimit().getValue() == ForwardLimitValue.ClosedToGround;
    }

    public Command intakeCoralCommand() {
        return this.setVelocityCommand(intakeCoralVelocity)
        .withName("IntakeCoral");
    }

    public Command intakeAlgaeCommand() {
        return this.setVelocityCommand(intakeAlgaeVelocity)
        .withName("IntakeAlgae");
    }

    public Command holdCommand() {
        return super.stopCommand()
        .withName("Hold");
    }

    public Command releaseCommand() {
        return this.setVelocityCommand(releaseVelocity)
        .withName("Release");
    }
}