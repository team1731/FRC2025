package frc.robot.subsystems.hand;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.state.StateMachineCallback;
import frc.robot.state.sequencer.SequenceInput;
import frc.robot.subsystems.ToggleableSubsystem;

public class HandIntakeSubsystem extends SubsystemBase implements ToggleableSubsystem {
    
    private TalonFX motor;
    private final NeutralOut brake = new NeutralOut();
    private StateMachineCallback scoreStateMachineCallback;
    private boolean intaking = false;
    private boolean stopping = false;
    private boolean watchingForScoreDetection = false;
    private double releaseStartedTime = 0;
    private double releaseRunningTime = 0;
    private double detectionStartedTime = 0;
    private double detectionRunningTime = 0;
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
        intaking = true;
    }

    public void intake(double velocity, StateMachineCallback callback) {
        scoreStateMachineCallback = callback;
        intake(velocity);
    }

    public void release(double velocity) {
       DutyCycleOut dutycycle = new DutyCycleOut(1);
       motor.setControl(dutycycle);
    }

    public void release(double velocity, double runningTime) {
        releaseStartedTime = Timer.getFPGATimestamp();
        releaseRunningTime = runningTime;
        release(velocity);
    }

    public void release(double velocity, double runningTime, StateMachineCallback callback) {
        scoreStateMachineCallback = callback;
        release(velocity, runningTime);
    }

    public void hold() {
        if(!enabled) return;
        DutyCycleOut dutyCycleOut = new DutyCycleOut(HandConstants.intakeHoldOutput * -1);
        motor.setControl(dutyCycleOut);
    }

    public void stop() {
        if(!enabled) return;
        intaking = false;
        motor.setControl(brake);
    }

    public void stop(StateMachineCallback callback) {
        scoreStateMachineCallback = callback;
        stopping = true;
        stop();
    }

    public void timedPieceDetection(double runningTime, StateMachineCallback callback) {
        scoreStateMachineCallback = callback;
        detectionStartedTime = Timer.getFPGATimestamp();
        detectionRunningTime = runningTime;
    }

    public void watchForScoreDetection(StateMachineCallback callback) {
        scoreStateMachineCallback = callback;
        watchingForScoreDetection = true;
    }

    public boolean pieceDetectionSwitchFlipped() {
        return motor.getReverseLimit().getValue() == ReverseLimitValue.ClosedToGround;
    }

    public boolean scoreDetectionSwitchFlipped() {
        return motor.getForwardLimit().getValue() == ForwardLimitValue.ClosedToGround;
    }

    

    /*
     * MOTOR INITIALIZATION
     */
    private void initializeMotor() {
        System.out.println("HandIntakeSubsystem: Starting UP & Initializing motor !!!!!!");
        motor = new TalonFX(HandConstants.intakeCanId, "rio");
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
        if(!enabled) return;

        if(releaseRunningTime != 0 && Timer.getFPGATimestamp() - releaseStartedTime >= releaseRunningTime) {
            stop();
            releaseRunningTime = 0;
            releaseStartedTime = 0;
            if(scoreStateMachineCallback != null) {
                System.out.println("HandIntakeSubsystem stopped release, should have finished shooting");
                scoreStateMachineCallback.setInput(SequenceInput.RELEASED_PIECE);
                scoreStateMachineCallback = null;
            }
        }

        if(intaking && pieceDetectionSwitchFlipped()) {
            intaking = false;
            if(scoreStateMachineCallback != null) {
                System.out.println("HandIntakeSubsystem reverse limit switch flipped, should have game piece");
                scoreStateMachineCallback.setInput(SequenceInput.DETECTED_PIECE);
                scoreStateMachineCallback = null;
            }
        }
        
        if(detectionRunningTime != 0 && Timer.getFPGATimestamp() - detectionStartedTime >= detectionRunningTime) {
            detectionRunningTime = 0;
            detectionStartedTime = 0;
            if(scoreStateMachineCallback != null) {
                System.out.println("Hand intake timer done, no piece detected");
                scoreStateMachineCallback.setInput(SequenceInput.TIMER_DONE);
                scoreStateMachineCallback = null;
            }
        }
            
        if(detectionRunningTime !=0 && pieceDetectionSwitchFlipped()){
            detectionRunningTime = 0;
            detectionStartedTime = 0;
            if(scoreStateMachineCallback != null) {
                System.out.println("HandIntakeSubsystem reverse limit switch flipped, should have game piece");
                scoreStateMachineCallback.setInput(SequenceInput.DETECTED_PIECE);
                scoreStateMachineCallback = null;
            }
        }
        
        if(watchingForScoreDetection && scoreDetectionSwitchFlipped()) {
            watchingForScoreDetection = false;
            if(scoreStateMachineCallback != null) {
                System.out.println("HandIntakeSubsystem forward limit switch flipped, should have scored");
                scoreStateMachineCallback.setInput(SequenceInput.SENSOR_SCORE);
                scoreStateMachineCallback = null;
            }
        }

        if(stopping && scoreStateMachineCallback != null) {
            stopping = false;
            System.out.println("HandIntakeSubsystem stopped intake");
            scoreStateMachineCallback.setInput(SequenceInput.STOPPED_INTAKE);
            scoreStateMachineCallback = null;
        }

        log();
    }


    /*
     * LOGGING
     */

    private void log() {
    }
}