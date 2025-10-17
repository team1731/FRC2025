package frc.robot.subsystems.arm;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.*;
import frc.lib.Utils;
import frc.lib.hardware.MotorIOTalonFX;
import frc.lib.subsystem.SingleMotorServoSubsystem;

public class ArmSubsystem extends SingleMotorServoSubsystem<MotorIOTalonFX> {
    public ArmSubsystem(boolean enabled) {
        super(enabled, ArmConstants.atPositionThreshold);
    }

    @Override
    protected void initializeHardware() {
        this.leadMotor = new MotorIOTalonFX(ArmConstants.armPortConfig);
        this.leadMotor.withCANCoder(
            ArmConstants.armCancoderDeviceId, 
            ArmConstants.armCANBus,
            ArmConstants.armCANCoderConfig
        );
        
        this.leadMotor.withPIDGains(ArmConstants.armPIDGains);

        this.leadMotor.withMotionProfile(ArmConstants.normalArmVelocity, ArmConstants.normalArmAcceleration, ArmConstants.armJerk);
        this.leadMotor.withStatorCurrentLimit(ArmConstants.armCurrentLimit);
        this.leadMotor.withFeedbackConfigs(ArmConstants.armFeedbackConfig);
        this.leadMotor.setNeutralMode(NeutralModeValue.Brake);

        this.leadMotor.applyConfigs();
    }

    @Override
    public void periodicTelemetry() {
        logger.log("Current Rotations", getRawRotations());
        logger.log("Target Rotations", getTargetRotations());
        logger.log("At Target Position", atTargetPosition());
    }

    public double getArmPosition() {
        return getRawRotations();
    }

    private Command setMotionMagicSpeedsCommand(double velocity, double acceleration) {
        return runOnce(() -> {
                leadMotor.setMotionMagicSpeeds(velocity, acceleration);
        });
    }

    private Command moveMotorCommand(double position) {
        return run(() -> setRotations(Utils.clamp(position, ArmConstants.minArmPosition, ArmConstants.maxArmPosition)))
        .until(() -> atTargetPosition());
    }

    public Command moveCommand(double position, boolean slowSpeed) {
        return Commands.either(
            setMotionMagicSpeedsCommand(ArmConstants.slowedArmVelocity, ArmConstants.slowedArmAcceleration), 
            setMotionMagicSpeedsCommand(ArmConstants.normalArmVelocity, ArmConstants.normalArmAcceleration),
            () -> slowSpeed
        ).andThen(moveMotorCommand(position))
        .withName("MoveArm" + (slowSpeed ? "NormalSpeed" : "SlowSpeed"));
    }

    public Command moveAlgaeCommand(double position) {
        return setMotionMagicSpeedsCommand(ArmConstants.slowedArmVelocity, ArmConstants.slowedArmAcceleration)
        .andThen(moveMotorCommand(position))
        .withName("MoveArmAlgaeSpeed");
    }

    public Command stopArmCommand() {
        return runOnce(() -> leadMotor.brake())
        .withName("StopArm");
    }
}