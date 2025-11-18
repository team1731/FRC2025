package frc.robot.subsystems.hand;

import frc.lib.frc1731.hardware.MotorIOTalonFX;
import frc.lib.frc1731.subsystem.PivotMotorSubsystem;
import frc.robot.Constants;

import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.subsystems.hand.HandConstants.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;

public class HandClamperSubsystem extends PivotMotorSubsystem<MotorIOTalonFX> {
    public HandClamperSubsystem(boolean enabled) {
        super(enabled);
        super.setTolerance(clamperPositionTolerance);
    }

    @Override
    protected void configureHardware() {
        this.motor = new MotorIOTalonFX(clamperPortConfig);
        this.motor.withCANCoder(clamperCancoderDeviceId, Constants.CANBUS_2_NAME, CANCoderConfigs);
        this.motor.withMotionMagicConfigs(mmConfigs);
        this.motor.withPIDGains(clamperGains);
        this.motor.withFeedbackConfigs(clamperFeedbackConfigs);
        this.motor.withStatorCurrentLimit(clamperStatorCurrentLimit);
        this.motor.applyConfigs();
    }

    @Override
    public void periodicTelemetry() {
        logger.log("Current Position", getPosition().in(Rotations));
        logger.log("Target Position", getTargetPosition().in(Rotations));
        logger.log("At Target Position", atTargetPosition());
    }
    
    public Command openCommand(Angle position) {
        return super.setPositionCommand(position)
        .withName("Open");
    }

    public Command closeCommand() {
        return openCommand(HandConstants.clamperHomePosition)
        .withName("Close");
    }

    public Command holdAlgaeCommand() {
        return openCommand(HandConstants.clamperAlgaePosition)
        .withName("HoldAlgae");
    }

    public Command holdCoralCommand() {
        return openCommand(HandConstants.clamperCoralPosition)
        .withName("HoldCoral");
    }

    public Command pluckAlgaeCommand() {
        return openCommand(HandConstants.clamperPluckAlgaePosition)
        .withName("PluckAlgae");
    }
    
    public Command reefAlgaeIntakeCommand() {
        return openCommand(HandConstants.clamperReefIntakePosition)
        .withName("ReefAlgaeIntake");
    }
}