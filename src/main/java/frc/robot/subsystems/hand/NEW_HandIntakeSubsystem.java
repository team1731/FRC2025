package frc.robot.subsystems.hand;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.ToggleableSubsystem;

public class NEW_HandIntakeSubsystem extends SubsystemBase implements ToggleableSubsystem {
    private TalonFX intakeMotor;
    private boolean isEnabled = false;
    private double targetVelocityRPM = 0.0;

    public NEW_HandIntakeSubsystem(boolean enabled) {
        this.isEnabled = enabled;
        if (!isEnabled) return;

        intakeMotor = new TalonFX(HandConstants.intakeCanId, "canivore2");
        TalonFXConfiguration configs = new TalonFXConfiguration();

        /* Voltage-based velocity requires a feed forward to account for the back-emf of the motor */
        configs.Slot0.kP = 0.22; // An error of 1 rotation per second results in 2V output
        configs.Slot0.kI = 0.5; // An error of 1 rotation per second increases output by 0.5V every second
        configs.Slot0.kD = 0.0001; // A change of 1 rotation per second squared results in 0.01 volts output
        configs.Slot0.kV = 0.12; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / Rotation per second

        // Peak output of 8 volts
        configs.Voltage.PeakForwardVoltage = 12;
        configs.Voltage.PeakReverseVoltage = -12;

        configs.MotorOutput.Inverted = HandConstants.intakeMotorDirection;

        var HWSwitchConfigs = new HardwareLimitSwitchConfigs();

        // Piece detection limit switch
        HWSwitchConfigs.ReverseLimitEnable = false;
        HWSwitchConfigs.ReverseLimitSource = ReverseLimitSourceValue.LimitSwitchPin;

        // Score detection limit switch
        // Not going to enable this limit switch, i.e., not going to affect motor stop/start
        HWSwitchConfigs.ForwardLimitEnable = false;
        HWSwitchConfigs.ForwardLimitSource = ForwardLimitSourceValue.LimitSwitchPin;

        // Add limit switch config
        configs.HardwareLimitSwitch = HWSwitchConfigs;

        configs.CurrentLimits.StatorCurrentLimit = 20;
        configs.CurrentLimits.StatorCurrentLimitEnable = true;


        /* Retry config apply up to 5 times, report if failure */
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = intakeMotor.getConfigurator().apply(configs);
            if (status.isOK()){
                break;
            }
        }

        if(!status.isOK()) {
            System.out.println("Could not apply configs, error code: " + status.toString());
        }
    }

    @Override
    public boolean isEnabled() {
        return isEnabled;
    }
    
    @Override
    public void periodic() {
        Logger.recordOutput("HandIntakeSubsystem/Current Velocity RPM", intakeMotor.getVelocity().getValueAsDouble());
        Logger.recordOutput("HandIntakeSubsystem/Target Velocity RPM", targetVelocityRPM);
        Logger.recordOutput("HandIntakeSubsystem/Piece Detection Switch Flipped", pieceDetectionSwitchFlipped());
        Logger.recordOutput("HandIntakeSubsystem/Score Detection Switch Flipped", scoreDetectionSwitchFlipped());
    }

    private boolean pieceDetectionSwitchFlipped() {
        return intakeMotor.getReverseLimit().getValue() == ReverseLimitValue.ClosedToGround;
    }

    private boolean scoreDetectionSwitchFlipped() {
        return intakeMotor.getForwardLimit().getValue() == ForwardLimitValue.ClosedToGround;
    }

    private void setVelocity(double desiredVelocity) {
        if (!isEnabled) return;
        this.targetVelocityRPM = desiredVelocity;
        intakeMotor.setControl(new VelocityVoltage(desiredVelocity));
    }

    public Command setVelocityCommand(double desiredVelocity) {
        return this.run(() -> {
            setVelocity(desiredVelocity);
        });
    }

    public Command intakeCoralCommand() {
        return this.setVelocityCommand(HandConstants.intakeCoralVelocity);
    }

    public Command intakeAlgaeCommand() {
        return this.setVelocityCommand(HandConstants.intakeAlgaeVelocity);
    }

    public Command holdCommand() {
        return this.runOnce(() -> {
            intakeMotor.setControl(new NeutralOut());
        });
    }
}