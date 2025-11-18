package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.frc1678.sim.PivotSim.PivotSimConstants;
import frc.lib.frc1731.hardware.MotorIOTalonFX;
import frc.lib.frc1731.subsystem.PivotMotorSubsystem;
import frc.robot.Constants;

public class ArmSubsystem extends PivotMotorSubsystem<MotorIOTalonFX> {
    public ArmSubsystem(boolean enabled) {
        super(enabled);
        super.setMotorTolerance(ArmConstants.atPositionThreshold);
        super.withSimulation(
                new PivotSimConstants()
                    .withMotor(DCMotor.getKrakenX60(1))
                    .withConstraints(
                        toMechanism(Rotations.of(-3)).in(Degrees), 
                        toMechanism(Rotations.of(26)).in(Degrees), 
                        ArmConstants.armHomePosition,
                        Units.inchesToMeters(21d)
                    ).withPhysics(1d/ArmConstants.armGearRationModifier, 0.19704, false)
            , ArmConstants.armPIDGains);
    }

    @Override
    protected void configureHardware() {
        this.motor = new MotorIOTalonFX(ArmConstants.armPortConfig);
        this.motor.withCANCoder(
            ArmConstants.armCancoderDeviceId, 
            Constants.CANBUS_NAME,
            ArmConstants.armCANCoderConfig
        );
        
        this.motor.withPIDGains(ArmConstants.armPIDGains);

        this.motor.withMotionProfile(ArmConstants.normalArmVelocity, ArmConstants.normalArmAcceleration, ArmConstants.armJerk);
        this.motor.withStatorCurrentLimit(ArmConstants.armCurrentLimit);
        this.motor.withFeedbackConfigs(ArmConstants.armFeedbackConfig);
        this.motor.setNeutralMode(NeutralModeValue.Brake);

        this.motor.applyConfigs();
    }

    @Override
    public void periodicTelemetry() {
        logger.log("Current Arm Degrees", getPosition().in(Degrees));
        logger.log("Target Arm Degrees", getTargetPosition().in(Degrees));
        logger.log("At Target Degrees", atTargetPosition());

        logger.log("Current Motor Rotations", getMotorPosition().in(Rotations));
        logger.log("Target Motor Rotations", getTargetMotorPosition().in(Rotations));
        logger.log("At Target Motor Rotations", getTargetMotorPosition().isNear(getMotorPosition(), getTolerance()));
    }

    public Command moveCommand(Angle position, boolean slowSpeed) {
        return Commands.either(
            setMotionProfileSpeeds(ArmConstants.slowedArmVelocity, ArmConstants.slowedArmAcceleration), 
            setMotionProfileSpeeds(ArmConstants.normalArmVelocity, ArmConstants.normalArmAcceleration),
            () -> slowSpeed
        ).andThen(setPositionCommand(position))
        .withName("MoveArm" + (slowSpeed ? "SlowSpeed" : "NormalSpeed"));
    }

    public Command moveCommand(double position, boolean slowSpeed) {
        return this.moveCommand(Degrees.of(position), slowSpeed);
    }
}