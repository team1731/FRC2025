package frc.robot.subsystems.hand;

import frc.lib.frc1731.Utils;
import frc.lib.frc1731.hardware.MotorIOTalonFX;
import frc.lib.frc1731.subsystem.SingleMotorServoSubsystem;
import frc.robot.Constants;

import static frc.robot.subsystems.hand.HandConstants.*;

import edu.wpi.first.wpilibj2.command.Command;

public class HandClamperSubsystem extends SingleMotorServoSubsystem<MotorIOTalonFX> {
    public HandClamperSubsystem(boolean enabled) {
        super(enabled, clamperPositionTolerance);
    }

    @Override
    protected void initializeHardware() {
        this.leadMotor = new MotorIOTalonFX(clamperPortConfig);
        this.leadMotor.withCANCoder(clamperCancoderDeviceId, Constants.CANBUS_2_NAME, CANCoderConfigs);
        this.leadMotor.withMotionMagicConfigs(mmConfigs);
        this.leadMotor.withPIDGains(clamperGains);
        this.leadMotor.withFeedbackConfigs(clamperFeedbackConfigs);
        this.leadMotor.withStatorCurrentLimit(clamperStatorCurrentLimit);
        this.leadMotor.applyConfigs();
    }

    @Override
    public void periodicTelemetry() {
        logger.log("Current Position", getRawRotations());
        logger.log("Target Position", getTargetRotations());
        logger.log("At Target Position", atTargetPosition());
    }

    public double getPosition() {
        return getRawRotations();
    }
    
    public Command openCommand(double position) {
        return super.setPositionCommand(Utils.clamp(position, HandConstants.minClamperPosition, HandConstants.maxClamperPosition))
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