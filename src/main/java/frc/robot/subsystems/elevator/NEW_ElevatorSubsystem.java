package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.*;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.*;
import frc.lib.Utils;
import frc.robot.subsystems.ToggleableSubsystem;
import frc.robot.subsystems.arm.ArmConstants;

@Deprecated(forRemoval = false)
public class NEW_ElevatorSubsystem extends SubsystemBase implements ToggleableSubsystem {
    private boolean isEnabled = false;
    private TalonFX masterMotor, followerMotor;
    private double targetPosition = 0.0;
    private DynamicMotionMagicVoltage mmReq = new DynamicMotionMagicVoltage(
            0,
            ElevatorConstants.normalElevatorVelocity, ElevatorConstants.normalElevatorAcceleration, ElevatorConstants.elevatorJerk);

    public NEW_ElevatorSubsystem(boolean enabled) {
        isEnabled = enabled;
        if (!isEnabled) return;

        masterMotor = new TalonFX(ElevatorConstants.elevatorCanId1, "canivore1");
        followerMotor = new TalonFX(ElevatorConstants.elevatorCanId2, "canivore1");
        followerMotor.setControl(new Follower(masterMotor.getDeviceID(), true));
        TalonFXConfiguration cfg = new TalonFXConfiguration();

        cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        /* Configure current limits */
        MotionMagicConfigs mm = cfg.MotionMagic;
        mm.MotionMagicCruiseVelocity = ElevatorConstants.normalElevatorVelocity; // 5 rotations per second cruise
        mm.MotionMagicAcceleration = ElevatorConstants.normalElevatorAcceleration; // Ta200ke approximately 0.5 seconds to reach max vel
        // Take approximately 0.2 seconds to reach max accel
        mm.MotionMagicJerk = ElevatorConstants.elevatorJerk;

        Slot0Configs slot0 = cfg.Slot0;
        slot0.kG = 0.1;
        slot0.kP = 4.9;
        slot0.kI = 0;
        slot0.kD = 0.0078125;
        slot0.kV = .14;
        slot0.kS = 0.02; // Approximately 0.25V to get the mechanism moving

        FeedbackConfigs fdb = cfg.Feedback;
        fdb.SensorToMechanismRatio = 1;

        // Apply the configs for Motor 1
        cfg.MotorOutput.Inverted = ElevatorConstants.elevatorMotor1Direction;
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = masterMotor.getConfigurator().apply(cfg);
            if (status.isOK())
                break;
        }

        if (!status.isOK()) {
            System.out.println("Could not configure device. Error: " + status.toString());
        }

        masterMotor.setPosition(0);

        StatusSignal<Angle> position = masterMotor.getPosition();
        StatusSignal<AngularVelocity> velocity = masterMotor.getVelocity();
        StatusSignal<Voltage> appliedVolts = masterMotor.getMotorVoltage();
        StatusSignal<Current> torqueCurrent = masterMotor.getTorqueCurrent();
        StatusSignal<Current> supplyCurrent = masterMotor.getSupplyCurrent();
        StatusSignal<Temperature> temp = masterMotor.getDeviceTemp();
        StatusSignal<Voltage> followerAppliedVolts = followerMotor.getMotorVoltage();
        StatusSignal<Current> followerTorqueCurrent = followerMotor.getTorqueCurrent();
        StatusSignal<Current> followerSupplyCurrent = followerMotor.getSupplyCurrent();
        StatusSignal<Temperature> followerTemp = followerMotor.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,
            position,
            velocity,
            appliedVolts,
            supplyCurrent,
            temp,
            followerAppliedVolts,
            followerTorqueCurrent,
            followerSupplyCurrent,
            followerTemp
        );

        torqueCurrent.setUpdateFrequency(250);
        ParentDevice.optimizeBusUtilizationForAll(masterMotor, followerMotor);
    }

    @Override
    public boolean isEnabled() {
        return isEnabled;
    }

    @Override
    public void periodic() {
        Logger.recordOutput("ElevatorSubsystem/Current Position", getElevatorPosition());
        Logger.recordOutput("ElevatorSubsystem/Target Position", this.targetPosition);
        Logger.recordOutput("ElevatorSubsystem/At Target Position", this.isAtTargetPosition());
    }

    public double getElevatorPosition() {
        if (!isEnabled) return 0;
        return masterMotor.getPosition().getValueAsDouble();
    }

    public boolean isAtPosition(double position) {
        return Math.abs(getElevatorPosition() - position) < ArmConstants.atPositionThreshold;
    }

    public boolean isAtTargetPosition() {
        return isAtPosition(targetPosition);
    }

    private void moveElevator(double position) {
        if(!isEnabled) return;

        double appliedPosition = 
            Utils.clamp(
                position * ElevatorConstants.gearRatioModifier, 
                ElevatorConstants.minElevatorPosition, 
                ElevatorConstants.maxElevatorPosition
            );
        targetPosition = appliedPosition;
        masterMotor.setControl(mmReq.withPosition(appliedPosition));
    }

    private void setMotionMagicSpeeds(double velocity, double acceleration) {
        mmReq.Velocity = velocity;
        mmReq.Acceleration = acceleration;
    }

    public Command moveElevatorSlowCommand(double targetPosition) {
        return new InstantCommand(() -> setMotionMagicSpeeds(ElevatorConstants.slowedElevatorVelocity, ElevatorConstants.slowedElevatorAcceleration))
        .andThen(this.run(() -> moveElevator(targetPosition))
        .until(() -> isAtTargetPosition()))
        .withName("MoveElevatorSlowSpeed");
    }

    public Command moveElevatorCommand(double targetPosition) {
        return new InstantCommand(() -> setMotionMagicSpeeds(ElevatorConstants.normalElevatorVelocity, ElevatorConstants.normalElevatorAcceleration))
        .andThen(this.run(() -> moveElevator(targetPosition))
        .until(() -> isAtTargetPosition()))
        .withName("MoveElevatorNormalSpeed");
    }

    public Command stopElevatorCommand() {
        return this.runOnce(() -> masterMotor.setControl(new NeutralOut()))
        .withName("StopElevator");
    }

    public Command unjamElevatorCommand() {
        return this.run(() -> {
            masterMotor.setControl(new DutyCycleOut(-0.1));
            masterMotor.setPosition(0.0);
        })
        .withName("UnjamElevator");
    }
}
