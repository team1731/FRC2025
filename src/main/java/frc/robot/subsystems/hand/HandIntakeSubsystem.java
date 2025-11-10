package frc.robot.subsystems.hand;

import frc.lib.frc1731.hardware.MotorIOTalonFX;
import frc.lib.frc1731.subsystem.SingleMotorVelocitySubsystem;
import edu.wpi.first.wpilibj2.command.Command;

import static frc.robot.subsystems.hand.HandConstants.*;

import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;

public class HandIntakeSubsystem extends SingleMotorVelocitySubsystem<MotorIOTalonFX> {
    public HandIntakeSubsystem(boolean enabled) {
        super(enabled);
    }

    @Override
    protected void initializeHardware() {
        this.leadMotor = new MotorIOTalonFX(intakePortConfig);
        this.leadMotor.withPIDGains(intakeGains);
        this.leadMotor.withVoltageConfigs(intakeVoltageConfigs);
        this.leadMotor.withHardwareLimitSwitchConfigs(intakeLimitSwitchConfigs);
        this.leadMotor.withStatorCurrentLimit(intakeStatorCurrentLimit);
        this.leadMotor.setNeutralMode(NeutralModeValue.Brake);
        this.leadMotor.applyConfigs();
    }

    @Override
    public void periodicTelemetry() {
        logger.log("Current Velocity RPM", getVelocityRPM());
        logger.log("Target Velocity RPM", getTargetVelocityRPM());
    }

    public boolean hasPiece() {
        return leadMotor.getReverseLimit().getValue() == ReverseLimitValue.ClosedToGround;
    }

    public boolean alignedToPole() {
        return leadMotor.getForwardLimit().getValue() == ForwardLimitValue.ClosedToGround;
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