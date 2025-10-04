package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.Utils;
import frc.robot.subsystems.ToggleableSubsystem;

public class NEW_ArmSubsystem extends SubsystemBase implements ToggleableSubsystem {
    private TalonFX armMotor;
    private CANcoder armCANcoder;
    private DynamicMotionMagicVoltage mmReq = new DynamicMotionMagicVoltage(
        0, ArmConstants.normalArmVelocity, ArmConstants.normalArmAcceleration, ArmConstants.armJerk);
    private boolean isEnabled = false;
    private double targetPosition = 0.0;

    public NEW_ArmSubsystem(boolean enabled) {
        isEnabled = enabled;
        armCANcoder = new CANcoder(ArmConstants.armCancoderDeviceId, "canivore2");
        CANcoderConfiguration cancoderConfigs = new CANcoderConfiguration();
        cancoderConfigs.MagnetSensor.MagnetOffset = -0.2138671875;     //-0.216552734375
        cancoderConfigs.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5; // TODO what should this be?
        cancoderConfigs.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        armCANcoder.getConfigurator().apply(cancoderConfigs);

        armMotor = new TalonFX(ArmConstants.armCanId, "canivore2");
        TalonFXConfiguration config = new TalonFXConfiguration();

        armMotor.getConfigurator().apply(config);
        
        /* Configure current limits */
        MotionMagicConfigs mm = config.MotionMagic;
        mm.MotionMagicCruiseVelocity = ArmConstants.normalArmVelocity; 
        mm.MotionMagicAcceleration = ArmConstants.normalArmAcceleration; 
        mm.MotionMagicJerk = ArmConstants.armJerk;

        Slot0Configs slot0 = config.Slot0;
        slot0.kP = 90;
        slot0.kI = 0;
        slot0.kD = 0.0099;
        slot0.kV = 0.9;
        slot0.kS = 0.02; // Approximately 0.25V to get the mechanism moving

        FeedbackConfigs fdb = config.Feedback;
        fdb.SensorToMechanismRatio = 1;
        fdb.FeedbackRemoteSensorID = armCANcoder.getDeviceID();;
        fdb.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        fdb.RotorToSensorRatio = 1600.0/18.0;
        config.CurrentLimits.StatorCurrentLimit = 40;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        //applies config to ArmMotor
        StatusCode status = StatusCode.StatusCodeNotInitialized;
               config.MotorOutput.Inverted = ArmConstants.armMotorDirection;
        for (int i = 0; i < 5; ++i) {
            status = armMotor.getConfigurator().apply(config);
            if (status.isOK())
                break;
        }
        
        if (!status.isOK()) {
            System.out.println("Could not configure device. Error: " + status.toString());
        }

        armMotor.setPosition(0);
        armMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    @Override
    public boolean isEnabled() {
        return this.isEnabled;
    }

    @Override
    public void periodic() {
        Logger.recordOutput("ArmSubsystem/Current Position", getArmPosition());
        Logger.recordOutput("ArmSubsystem/Target Position", this.targetPosition);
        Logger.recordOutput("ArmSubsystem/At Target Position", this.isAtTargetPosition());
    }

    public double getArmPosition(){
        if (!isEnabled) return 0;
        return armMotor.getPosition().getValueAsDouble();
    }

    public boolean isAtPosition(double position){
        return Math.abs(getArmPosition() - position) < ArmConstants.atPositionThreshold;
    }

    public boolean isAtTargetPosition() {
        return isAtPosition(targetPosition);
    }

    private void moveArm(double position) {
        if (!isEnabled) return;

        double appliedPosition = Utils.clamp(
            position * ArmConstants.armGearRationModifier, 
            ArmConstants.minArmPosition, 
            ArmConstants.maxArmPosition
        );
        targetPosition = appliedPosition;
        armMotor.setControl(mmReq.withPosition(appliedPosition).withFeedForward(0.0));
    }

    private void setMotionMagicSpeeds(double velocity, double acceleration) {
        mmReq.Velocity = velocity;
        mmReq.Acceleration = acceleration;
    }

    public Command moveArmSlowSpeed(double position) {
        return new InstantCommand(() -> setMotionMagicSpeeds(ArmConstants.slowedArmVelocity, ArmConstants.slowedArmAcceleration), this)
            .andThen(this.run(() -> moveArm(position)))
            .until(() -> isAtPosition(position))
            .withName("MoveArmSlowSpeed");
    }

    public Command moveArmNormalSpeed(double position) {
        return new InstantCommand(() -> setMotionMagicSpeeds(ArmConstants.normalArmVelocity, ArmConstants.normalArmAcceleration), this)
            .andThen(this.run(() -> moveArm(position)))
            .until(() -> isAtTargetPosition())
            .withName("MoveArmNormalSpeed");
    }
}