package frc.robot.subsystems.score;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.ToggleableSubsystem;

public class HandIntakeSubsystem extends SubsystemBase implements ToggleableSubsystem {
    
    private TalonFX motor;
    private final NeutralOut brake = new NeutralOut();
    private boolean enabled;
    
    
    @Override
    public boolean isEnabled() {
        return enabled;
    }

    public HandIntakeSubsystem(boolean enabled) {
        this.enabled = enabled;
        if(!enabled) return;
        initializeMotor();
    }

    /*
     * MOTOR MOVEMENT
     */

    public void intake(double velocity) {
        VelocityDutyCycle velocityDutyCycle = new VelocityDutyCycle(velocity * -1); // velocity, reverse motor direction
        motor.setControl(velocityDutyCycle);
    }

    public void release(double velocity) {
        VelocityDutyCycle velocityDutyCycle = new VelocityDutyCycle(velocity);
        motor.setControl(velocityDutyCycle);
    }

    public void stop() {
        if(!enabled) return;
        motor.setControl(brake);
    }

    

    /*
     * MOTOR INITIALIZATION
     */
    private void initializeMotor() {
        System.out.println("HandIntakeSubsystem: Starting UP & Initializing motor !!!!!!");
        motor = new TalonFX(HandConstants.handIntakeCanId, Constants.CANBUS_NAME);
        TalonFXConfiguration configs = new TalonFXConfiguration();

        /* Voltage-based velocity requires a feed forward to account for the back-emf of the motor */
        configs.Slot0.kP = 0.22; // An error of 1 rotation per second results in 2V output
        configs.Slot0.kI = 0.5; // An error of 1 rotation per second increases output by 0.5V every second
        configs.Slot0.kD = 0.0001; // A change of 1 rotation per second squared results in 0.01 volts output
        configs.Slot0.kV = 0.12; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / Rotation per second

        // Peak output of 8 volts
        configs.Voltage.PeakForwardVoltage = 12;
        configs.Voltage.PeakReverseVoltage = -12;

        configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        /* Retry config apply up to 5 times, report if failure */
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = motor.getConfigurator().apply(configs);
            if (status.isOK()){
                break;
            }
        }

        if(!status.isOK()) {
            System.out.println("Could not apply configs, error code: " + status.toString());
        }
    }


    /*
     * MOTOR STATUS/PROGRESS TRACKING
     */

    public void periodic() {
        if (!enabled) return;

        log();
    }


    /*
     * LOGGING
     */

    private void log() {
    }
}
