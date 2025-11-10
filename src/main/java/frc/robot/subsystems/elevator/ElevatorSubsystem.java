package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.signals.*;

import edu.wpi.first.wpilibj2.command.*;
import frc.lib.Utils;
import frc.lib.hardware.MotorIOTalonFX;
import frc.lib.subsystem.CoupledMotorServoSubsystem;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;

public class ElevatorSubsystem extends CoupledMotorServoSubsystem<MotorIOTalonFX> {
    public ElevatorSubsystem(boolean enabled) {
        super(new MotorIOTalonFX(followerPortConfig), enabled, atPositionTolerance);
        ParentDevice.optimizeBusUtilizationForAll(leadMotor.getMotor(), followerMotor.getMotor());
    }

    @Override
    protected void initializeHardware() {
        this.leadMotor = new MotorIOTalonFX(leadPortConfig);
        this.leadMotor.withFeedbackConfigs(feedbackConfigs);
        this.leadMotor.withPIDGains(motionMagicGains);
        this.leadMotor.withMotionProfile(normalElevatorVelocity, normalElevatorAcceleration, elevatorJerk);
        this.leadMotor.setNeutralMode(NeutralModeValue.Brake);
        this.leadMotor.resetEncoderPosition(0d);
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

    public boolean atTargetPosition() {
        return super.atTargetPosition();
    }

    public Command moveCommand(double position, boolean slowed) {
        return new InstantCommand(() -> {
            if (slowed) {
                leadMotor.withMotionProfile(slowedElevatorVelocity, slowedElevatorAcceleration, elevatorJerk);
            } else {
                leadMotor.withMotionProfile(normalElevatorVelocity, normalElevatorAcceleration, elevatorJerk);
            }
        }).andThen(
            super.setPositionCommand(Utils.clamp(position, minElevatorPosition, maxElevatorPosition))
            .until(() -> atTargetPosition())
        ).withName("MoveElevator" + (slowed?"Slow":""));
    }

    public Command homeCommand() {
        return moveCommand(elevatorHomePosition, false)
        .withName("Home");
    }

    public Command unjamCommand() {
        return run(() -> {
            leadMotor.setPercentOutput(-0.1);
            leadMotor.resetEncoderPosition(0);
        })
        .withName("Unjam");
    }

    public Command stopCommand() {
        return runOnce(() -> leadMotor.brake())
        .withName("Stop");
    }
}