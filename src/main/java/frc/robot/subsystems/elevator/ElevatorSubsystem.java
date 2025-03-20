package frc.robot.subsystems.elevator;

import java.util.ResourceBundle.Control;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.state.StateMachineCallback;
import frc.robot.state.sequencer.SequenceInput;
import frc.robot.subsystems.ToggleableSubsystem;

public class ElevatorSubsystem extends SubsystemBase implements ToggleableSubsystem {

    // motors for the elevator
    private TalonFX elevatorMotor1;
    private TalonFX elevatorMotor2;
    private double desiredPosition;

    // motor movement
    private double arbitraryFeedForward = 0;
    private DynamicMotionMagicVoltage mmReq = new DynamicMotionMagicVoltage(
        0, ElevatorConstants.normalElevatorVelocity, ElevatorConstants.normalElevatorAcceleration, ElevatorConstants.elevatorJerk);
    private final NeutralOut brake = new NeutralOut();

    // state machine callback handling
    private StateMachineCallback stateMachineCallback;
    private boolean callbackOnDone = false;
    private boolean callbackOnThreshold = false;
    private double positionThreshold = 0;
    private boolean raisingThreshold = false;
    private boolean unStuckElevator = false;

    private boolean enabled;
    
    @Override
    public boolean isEnabled() {
        return enabled;
    }

    public ElevatorSubsystem(boolean enabled) {
        this.enabled = enabled;
        if (!enabled)
            return;
        initializeElevatorMotors();
    }
    
    /*
     * Elevator MOTOR MOVEMENT
     */

    private void moveElevator(double position) {
        if(!enabled) return;

        // do not go outside boundary thresholds
        if(position * ElevatorConstants.gearRatioModifier > ElevatorConstants.maxElevatorPosition) {
            desiredPosition = ElevatorConstants.maxElevatorPosition;
        } else if(position * ElevatorConstants.gearRatioModifier < ElevatorConstants.minElevatorPosition) {
            desiredPosition = ElevatorConstants.minElevatorPosition;
        } else {
            desiredPosition = position * ElevatorConstants.gearRatioModifier;
        }

        elevatorMotor1.setControl(mmReq.withPosition(desiredPosition).withFeedForward(arbitraryFeedForward));
        elevatorMotor2.setControl(mmReq.withPosition(desiredPosition).withFeedForward(arbitraryFeedForward));
       
    }

    public void moveElevatorNormalSpeed(double position) {
        mmReq.Velocity = ElevatorConstants.normalElevatorVelocity;
        mmReq.Acceleration = ElevatorConstants.normalElevatorAcceleration;
        moveElevator(position);
    }

    public void moveElevatorSlowSpeed(double position) {
        mmReq.Velocity = ElevatorConstants.slowedElevatorVelocity;
        mmReq.Acceleration = ElevatorConstants.slowedElevatorAcceleration;
        moveElevator(position);
    }

    public void moveElevatorNormalSpeed(double position, StateMachineCallback callback) {
        stateMachineCallback = callback;
        callbackOnDone = true;
        moveElevatorNormalSpeed(position);
    }

    public void moveElevatorNormalSpeed(double position, StateMachineCallback callback, double threshold) {
        stateMachineCallback = callback;
        callbackOnDone = true;
        callbackOnThreshold = true;
        positionThreshold = threshold * ElevatorConstants.gearRatioModifier;
        raisingThreshold = threshold < position;
        moveElevatorNormalSpeed(position);
    }

    public void moveElevatorSlowSpeed(double position, StateMachineCallback callback) {
        stateMachineCallback = callback;
        callbackOnDone = true;
        moveElevatorSlowSpeed(position);
    }

    public void stopElevator() {
        if(!enabled) return;
        elevatorMotor1.setControl(brake);
        elevatorMotor2.setControl(brake);
    }

    // Initialize Motors
    private void initializeElevatorMotors() {
        if (!enabled)
            return;

        System.out.println("elevatorSubsystem: Starting UP & Initializing elevator motors !!!!!!");
        elevatorMotor1 = new TalonFX(ElevatorConstants.elevatorCanId1, "canivore1");
        elevatorMotor2 = new TalonFX(ElevatorConstants.elevatorCanId2, "canivore1");
        TalonFXConfiguration cfg = new TalonFXConfiguration();
        elevatorMotor1.getConfigurator().apply(cfg);
        elevatorMotor2.getConfigurator().apply(cfg);

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
            status = elevatorMotor1.getConfigurator().apply(cfg);
            if (status.isOK())
                break;
        }
        if (!status.isOK()) {
            System.out.println("Could not configure device. Error: " + status.toString());
        }

        // Apply the configs for Motor 2
        cfg.MotorOutput.Inverted = ElevatorConstants.elevatorMotor2Direction;
        status = StatusCode.StatusCodeNotInitialized;
         
        for (int i = 0; i < 5; ++i) {
            status = elevatorMotor2.getConfigurator().apply(cfg);
            if (status.isOK())
                break;
        }
        if (!status.isOK()) {
            System.out.println("Could not configure device. Error: " + status.toString());
        }
        

        elevatorMotor1.setPosition(0);
        elevatorMotor2.setPosition(0);
        elevatorMotor1.setNeutralMode(NeutralModeValue.Brake);
        elevatorMotor2.setNeutralMode(NeutralModeValue.Brake);

    }

    public void periodic() {
        if(!enabled) return;

        /*
         * Score State Machine callback handling
         */
        if(callbackOnDone && isAtPosition(desiredPosition) && stateMachineCallback != null) {
            // final position reached, notify the state machine
            callbackOnDone = false;
            System.out.println("Elevator subsystem callback: " + getElevatorPosition());
            stateMachineCallback.setInput(SequenceInput.ELEVATOR_DONE);
        } else if(callbackOnThreshold && stateMachineCallback != null) {
            // check to see if the threshold was met, if so notify the state machine
            boolean thresholdMet = raisingThreshold && getElevatorPosition() >= positionThreshold ||
                !raisingThreshold && getElevatorPosition() <= positionThreshold;
            if(thresholdMet) {
                System.out.println("Elevator subsystem threshold callback: " + getElevatorPosition());
                stateMachineCallback.setInput(SequenceInput.ELEVATOR_THRESHOLD_MET);
                callbackOnThreshold = false;
                positionThreshold = 0;
            }
        }

        if(unStuckElevator){
            DutyCycleOut dutycycle = new DutyCycleOut(-0.1);
            elevatorMotor1.setControl(dutycycle);
            elevatorMotor2.setControl(dutycycle);
            elevatorMotor1.setPosition(0);
            elevatorMotor2.setPosition(0);
        };

        log();
    }

    public double getElevatorPosition() {
        if (!enabled)
            return 0;
        return elevatorMotor1.getPosition().getValueAsDouble();
    }

    public boolean isAtPosition(double position) {
        double tolerance = 1;
        return Math.abs(getElevatorPosition() - position) < tolerance;
    }

    public boolean setElevatorUnstuck(boolean unstuck){
        unStuckElevator = unstuck;
        return unStuckElevator;  
    };

    private void log(){
        SmartDashboard.putNumber("elevator motor 1 position", elevatorMotor1.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("elevator motor 2 position", elevatorMotor2.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("elevator desired position", desiredPosition);
        SmartDashboard.putNumber("elevator motor 1 closedLoopError",
                elevatorMotor1.getClosedLoopError().getValueAsDouble());
        SmartDashboard.putNumber("elevator motor 1 closedLoopError",
                elevatorMotor1.getClosedLoopError().getValueAsDouble());
        SmartDashboard.putNumber("elevator motor 1 closedLoopReference",
                elevatorMotor1.getClosedLoopReference().getValueAsDouble());
        SmartDashboard.putNumber("elevator motor 2 closedLoopReference",
                elevatorMotor2.getClosedLoopReference().getValueAsDouble());
        SmartDashboard.putNumber("elevator motor 1 closedLoopOutput",
                elevatorMotor1.getClosedLoopOutput().getValueAsDouble());
        SmartDashboard.putNumber("elevator motor 2 closedLoopOutput",
                elevatorMotor2.getClosedLoopOutput().getValueAsDouble());
        SmartDashboard.putNumber("elevator motor 1 statorCurrent",
                elevatorMotor1.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("elevator motor 2 statorCurrent",
                elevatorMotor2.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Arbitrary Feed Forward", arbitraryFeedForward);
    }
}