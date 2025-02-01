package frc.robot.subsystems.score;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.state.StateMachineCallback;
import frc.robot.state.score.ScoreInput;
import frc.robot.subsystems.ToggleableSubsystem;

public class ElevatorSubsystem extends SubsystemBase implements ToggleableSubsystem {

    // motors for the elevator
    private TalonFX elevatorMotor1;
    private TalonFX elevatorMotor2;
    private double desiredPosition;
    private double arbitraryFeedForward;
    private MotionMagicVoltage mmReq1 = new MotionMagicVoltage(0);
    // private MotionMagicVoltage mmReq2;
    // private final VelocityVoltage m_voltageVelocity = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);

    private boolean enabled;
    private double ampTimeStarted;
    private int TEST_ONLY_COUNTER_REMOVE_ME;

    
    private StateMachineCallback scoreStateMachineCallback;
    
    
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

    public void moveElevator(double position) {
        if(!enabled) return;
        desiredPosition = position;
        elevatorMotor1.setControl(mmReq1.withPosition(desiredPosition).withFeedForward(arbitraryFeedForward));
        elevatorMotor2.setControl(mmReq1.withPosition(desiredPosition).withFeedForward(arbitraryFeedForward));
       
    }

    public void moveElevator(double position, StateMachineCallback callback){
        scoreStateMachineCallback = callback;
        moveElevator(position);
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

        elevatorMotor1.setNeutralMode(NeutralModeValue.Brake);
        elevatorMotor2.setNeutralMode(NeutralModeValue.Brake);

        /* Configure current limits */
        MotionMagicConfigs mm = cfg.MotionMagic;
        mm.MotionMagicCruiseVelocity = 70; // 5 rotations per second cruise
        mm.MotionMagicAcceleration = 250; // Ta200ke approximately 0.5 seconds to reach max vel
        // Take approximately 0.2 seconds to reach max accel
        mm.MotionMagicJerk = 0;

        Slot0Configs slot0 = cfg.Slot0;
        slot0.kP = 4.9;
        slot0.kI = 0;
        slot0.kD = 0.0078125;
        slot0.kV = 0.009375;
        slot0.kS = 0.02; // Approximately 0.25V to get the mechanism moving

        FeedbackConfigs fdb = cfg.Feedback;
        fdb.SensorToMechanismRatio = 1;

        // Apply the configs for Motor 1
        cfg.MotorOutput.Inverted = ElevatorConstants.elevatorDirection;
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
        cfg.MotorOutput.Inverted = ElevatorConstants.elevatorDirection;
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
        if (!enabled) return;

        if (isAtPosition(desiredPosition) && scoreStateMachineCallback != null){
            scoreStateMachineCallback.setInput(ScoreInput.ELEVATOR_DONE);
            scoreStateMachineCallback = null;
        }

        log();
    }

    public double getElevatorPosition() {
        if (!enabled)
            return 0;
        return elevatorMotor1.getPosition().getValueAsDouble();
    }

    public boolean isAtPosition(double position) {
        double tolerance = 2;
        return Math.abs(getElevatorPosition() - position) < tolerance;
    }

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