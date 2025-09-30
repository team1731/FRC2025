package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.ToggleableSubsystem;

public class NEW_ElevatorSubsystem extends SubsystemBase implements ToggleableSubsystem {
    private boolean isEnabled = false;
    private TalonFX masterMotor, followerMotor;
    private double targetPosition = 0.0;
    private DynamicMotionMagicVoltage mmReq = new DynamicMotionMagicVoltage(
            0,
            ElevatorConstants.normalElevatorVelocity, ElevatorConstants.normalElevatorAcceleration, ElevatorConstants.elevatorJerk);

    public NEW_ElevatorSubsystem(boolean enabled) {
        isEnabled = enabled;
    }

    @Override
    public boolean isEnabled() {
        return isEnabled;
    }
    
    // Initialize Motors
    private void initializeElevatorMotors() {
        if (!isEnabled) return;

        // System.out.println("elevatorSubsystem: Starting UP & Initializing elevator motors !!!!!!");
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
        StatusSignal<Current> torqueCurrent = masterMotor.getTorqueCurrent();;
        StatusSignal<Current> supplyCurrent = masterMotor.getSupplyCurrent();;
        StatusSignal<Temperature> temp = masterMotor.getDeviceTemp();;
        StatusSignal<Voltage> followerAppliedVolts = followerMotor.getMotorVoltage();;
        StatusSignal<Current> followerTorqueCurrent = followerMotor.getTorqueCurrent();;
        StatusSignal<Current> followerSupplyCurrent = followerMotor.getSupplyCurrent();;
        StatusSignal<Temperature> followerTemp = followerMotor.getDeviceTemp();;

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
}
