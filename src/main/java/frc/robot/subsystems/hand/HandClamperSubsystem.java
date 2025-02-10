package frc.robot.subsystems.hand;

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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.state.StateMachineCallback;
import frc.robot.state.sequencer.SequenceInput;
import frc.robot.subsystems.ToggleableSubsystem;

public class HandClamperSubsystem extends SubsystemBase implements ToggleableSubsystem {

    private TalonFX motor;
    private CANcoder clamperCancoder;
    private double desiredPosition;
    private double arbitraryFeedForward = 0;
    private MotionMagicVoltage mmReq1 = new MotionMagicVoltage(0);
    private boolean enabled;
    private StateMachineCallback scoreStateMachineCallback;
    
    
    @Override
    public boolean isEnabled() {
        return enabled;
    }

    public HandClamperSubsystem(boolean enabled) {
        this.enabled = enabled;
        if(!enabled) return;
        initializeMotor();
    }
    
    /*
     * GRIP MOTOR MOVEMENT
     */

    private void moveHand(double position) {
        if(!enabled) return;

        // do not go outside boundary thresholds
        if(position > HandConstants.maxClamperPosition) {
            desiredPosition = HandConstants.maxClamperPosition;
        } else if(position < HandConstants.minClamperPosition) {
            desiredPosition = HandConstants.minClamperPosition;
        } else {
            desiredPosition = position;
        }

        motor.setControl(mmReq1.withPosition(desiredPosition).withFeedForward(arbitraryFeedForward));
    }

    public void open(double position) {
        moveHand(position);
    }

    public void open(double position, StateMachineCallback callback) {
        scoreStateMachineCallback = callback;
        moveHand(position);
    }

    public void close() {
        moveHand(HandConstants.clamperHomePosition);
    }

    public void close(StateMachineCallback callback) {
        scoreStateMachineCallback = callback;
        close();
    }
    

    /*
     * MOTOR INITIALIZATION
     */
    private void initializeMotor() {
        System.out.println("HandSubsystem: Starting UP & Initializing intake motor !!!!!!");

        clamperCancoder = new CANcoder(HandConstants.clamperCancoderDeviceId, "rio");
        CANcoderConfiguration cancoderConfigs = new CANcoderConfiguration();
        cancoderConfigs.MagnetSensor.MagnetOffset =0.2841796875;
        clamperCancoder.getConfigurator().apply(cancoderConfigs);

        motor = new TalonFX(HandConstants.clamperCanId, "rio");
        TalonFXConfiguration cfg = new TalonFXConfiguration();
        motor.getConfigurator().apply(cfg);

        /* Configure current limits */
        MotionMagicConfigs mm = cfg.MotionMagic;
        mm.MotionMagicCruiseVelocity = 70; // 5 rotations per second cruise
        mm.MotionMagicAcceleration = 250; // Ta200ke approximately 0.5 seconds to reach max vel
        // Take approximately 0.2 seconds to reach max accel
        mm.MotionMagicJerk = 0;

        Slot0Configs slot0 = cfg.Slot0;
        slot0.kP = 60;  
        slot0.kI = 0;
        slot0.kD = 0.0078125;
        slot0.kV = 0.009375;
        slot0.kS = 0.02; // Approximately 0.25V to get the mechanism moving

        FeedbackConfigs fdb = cfg.Feedback;
        fdb.FeedbackRemoteSensorID = clamperCancoder.getDeviceID();
        fdb.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        fdb.RotorToSensorRatio = 125;
        fdb.SensorToMechanismRatio = 1;
        cfg.CurrentLimits.StatorCurrentLimit = 40;
        cfg.CurrentLimits.StatorCurrentLimitEnable = true;

        // Apply the config changes
        cfg.MotorOutput.Inverted = HandConstants.clamperMotorDirection;
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = motor.getConfigurator().apply(cfg);
            if (status.isOK())
                break;
        }
        if (!status.isOK()) {
            System.out.println("Could not configure device. Error: " + status.toString());
        }
        
        motor.setNeutralMode(NeutralModeValue.Brake);
    }


    /*
     * MOTOR STATUS/PROGRESS TRACKING
     */

    public void periodic() {
        if (!enabled) return;

        if(isAtPosition(desiredPosition) && scoreStateMachineCallback != null){
            scoreStateMachineCallback.setInput(SequenceInput.HAND_DONE);
            scoreStateMachineCallback = null;
        }

        log();
    }

    public double getPosition() {
        if(!enabled) return 0;
        return motor.getPosition().getValueAsDouble();
    }

    public boolean isAtPosition(double position) {
        double tolerance = HandConstants.clamperPositionTolerance;
        return Math.abs(getPosition() - position) < tolerance;
    }


    /*
     * LOGGING
     */

    private void log() {
        SmartDashboard.putNumber("hand motor position", motor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("hand desired position", desiredPosition);
        SmartDashboard.putNumber("hand motor closedLoopError", motor.getClosedLoopError().getValueAsDouble());
        SmartDashboard.putNumber("hand motor closedLoopError", motor.getClosedLoopError().getValueAsDouble());
        SmartDashboard.putNumber("hand motor closedLoopReference", motor.getClosedLoopReference().getValueAsDouble());
        SmartDashboard.putNumber("hand motor closedLoopOutput", motor.getClosedLoopOutput().getValueAsDouble());
        SmartDashboard.putNumber("hand motor statorCurrent", motor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Arbitrary Feed Forward", arbitraryFeedForward);
    }
}