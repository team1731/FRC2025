package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.signals.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.*;
import frc.lib.frc1731.hardware.MotorIOTalonFX;
import frc.lib.frc1731.subsystem.LinearMotorSubsystem;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.elevator.ElevatorConstants.*;

public class ElevatorSubsystem extends LinearMotorSubsystem<MotorIOTalonFX> {
    public ElevatorSubsystem(boolean enabled) {
        super(enabled, Inches.of(1d), gearRatioModifier);
        super.setTolerance(Rotations.of(atPositionTolerance));
        super.withSimulation(simConstants, motionMagicGains);
    }

    @Override
    protected void configureHardware() {
        this.motor = new MotorIOTalonFX(leadPortConfig);
        this.motor.withFeedbackConfigs(feedbackConfigs);
        this.motor.withPIDGains(motionMagicGains);
        this.motor.withMotionProfile(normalElevatorVelocity, normalElevatorAcceleration, 0d);
        this.motor.setNeutralMode(NeutralModeValue.Brake);
        this.motor.resetEncoderPosition(0d);

        MotorIOTalonFX follower = new MotorIOTalonFX(followerPortConfig);
        this.motor.withFollower(follower, true);
        ParentDevice.optimizeBusUtilizationForAll(motor.getMotor(), follower.getMotor());
    }

    @Override
    public void periodicTelemetry() {
        logger.log("Current Position", getPosition().in(Inches));
        logger.log("Target Position", getTargetPosition().in(Inches));
        logger.log("At Target Position", atTargetPosition());
        logger.log("Current Motor Position", getMotorRotations().in(Rotations));
        logger.log("Target Motor Position", getTargetMotorPosition().in(Rotations));
    }

    public Command setPositionCommand(Angle position, boolean slowed) {
        return Commands.either(
            setMotionProfileSpeeds(slowedElevatorVelocity, slowedElevatorAcceleration),
            setMotionProfileSpeeds(normalElevatorVelocity, normalElevatorAcceleration), 
            () -> slowed
        ).andThen(super.setPositionCommand(position))
        .withName("SetElevatorPosition" + (slowed?"Slow":""));
    }

    public Command setPositionCommand(Distance position, boolean slowed) {
        return Commands.either(
            setMotionProfileSpeeds(slowedElevatorVelocity, slowedElevatorAcceleration),
            setMotionProfileSpeeds(normalElevatorVelocity, normalElevatorAcceleration), 
            () -> slowed
        ).andThen(super.setPositionCommand(position))
        .withName("SetElevatorPosition" + (slowed?"Slow":""));
    }

    public Command homeCommand() {
        return setPositionCommand(elevatorHomePosition, false)
        .withName("Home");
    }

    public Command unjamCommand() {
        return run(() -> {
            motor.setPercentOutput(-0.1);
            motor.resetEncoderPosition(0);
        })
        .withName("Unjam");
    }

    public Command stopCommand() {
        return runOnce(() -> motor.brake())
        .withName("Stop");
    }
}