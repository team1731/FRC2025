package frc.robot.subsystems.hand;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.Utils;
import frc.robot.subsystems.ToggleableSubsystem;

public class NEW_HandClamperSubsystem extends SubsystemBase implements ToggleableSubsystem {
    private boolean isEnabled = false;
    private TalonFX clampMotor;
    private CANcoder clampCancoder;
    private double targetPosition = 0.0;
    private MotionMagicVoltage mmReq1 = new MotionMagicVoltage(0);

    public NEW_HandClamperSubsystem(boolean enabled) {
        this.isEnabled = enabled;
        if (!isEnabled) return;

        clampCancoder = new CANcoder(HandConstants.clamperCancoderDeviceId, "canivore2");
        CANcoderConfiguration cancoderConfigs = new CANcoderConfiguration();
        cancoderConfigs.MagnetSensor.MagnetOffset = 0.195556640625; //0.2854;  //0.248291015625
        cancoderConfigs.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        clampCancoder.getConfigurator().apply(cancoderConfigs);

        clampMotor = new TalonFX(HandConstants.clamperCanId, "canivore2");
        TalonFXConfiguration cfg = new TalonFXConfiguration();
        clampMotor.getConfigurator().apply(cfg);

        /* Configure current limits */
        MotionMagicConfigs mm = cfg.MotionMagic;
        mm.MotionMagicCruiseVelocity = 70; // should be more like 70.0/125.0; // 5 rotations per second cruise
        mm.MotionMagicAcceleration = 250; // should be more like140.0/125.0; // Ta200ke approximately 0.5 seconds to reach max vel
        // Take approximately 0.2 seconds to reach max accel
        mm.MotionMagicJerk = 0;

        Slot0Configs slot0 = cfg.Slot0;
        slot0.kP = 150;  // seems like this should be more like 612
        slot0.kI = 0;
        slot0.kD = 0.0078125;      
        slot0.kV = 0.009375;
        slot0.kS = 0.02; // Approximately 0.25V to get the mechanism moving

        FeedbackConfigs fdb = cfg.Feedback;
        fdb.FeedbackRemoteSensorID = clampCancoder.getDeviceID();
        fdb.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        fdb.RotorToSensorRatio = 125;
        fdb.SensorToMechanismRatio = 1;
        cfg.CurrentLimits.StatorCurrentLimit = 40;
        cfg.CurrentLimits.StatorCurrentLimitEnable = true;

        // Apply the config changes
        cfg.MotorOutput.Inverted = HandConstants.clamperMotorDirection;
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = clampMotor.getConfigurator().apply(cfg);
            if (status.isOK())
                break;
        }
        if (!status.isOK()) {
            System.out.println("Could not configure device. Error: " + status.toString());
        }
        
        clampMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    @Override
    public boolean isEnabled() {
        return isEnabled;
    }
    
    @Override
    public void periodic() {
        Logger.recordOutput("HandClamperSubsystem/Current Position", getPosition());
        Logger.recordOutput("HandClamperSubsystem/Target Position", targetPosition);
        Logger.recordOutput("HandClamperSubsystem/At Target Position", isAtTargetPosition());
    }

    public double getPosition() {
        if(!isEnabled) return 0;
        return clampMotor.getPosition().getValueAsDouble();
    }

    public boolean isAtPosition(double position) {
        double tolerance = HandConstants.clamperPositionTolerance;
        return Math.abs(getPosition() - position) < tolerance;
    }

    public boolean isAtTargetPosition() {
        return isAtPosition(targetPosition);
    }

    private void moveHand(double desiredPosition) {
        if (!isEnabled) return;
        this.targetPosition = desiredPosition;
        double appliedPosition = Utils.clamp(targetPosition, HandConstants.minClamperPosition, HandConstants.maxClamperPosition);
        clampMotor.setControl(mmReq1.withPosition(appliedPosition));
    }

    public Command moveHandCommand(double desiredPosition) {
        return this.run(() -> moveHand(desiredPosition))
            .until(() -> isAtTargetPosition())
            .withName("MoveHand");
    }

    public Command closeCommand() {
        return moveHandCommand(HandConstants.clamperHomePosition)
            .withName("CloseHand");
    }
}