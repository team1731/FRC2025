package frc.robot.subsystems.arm;

import java.security.PublicKey;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.state.StateMachineCallback;
import frc.robot.state.score.ScoreInput;
import frc.robot.subsystems.ToggleableSubsystem;



public class ArmSubsystem extends SubsystemBase implements ToggleableSubsystem{
    
    
    private TalonFX armMotor;
    private MotionMagicVoltage mmReq = new MotionMagicVoltage(0);

    private double desiredPosition;
    private boolean enabled;
    private double arbitraryFeedForward = 0;

    private StateMachineCallback scoreStateMachineCallback;


    @Override
    public boolean isEnabled() {
        return enabled;
    }

    public ArmSubsystem(boolean enabled) {
        this.enabled = enabled;
        if (!enabled)
            return;
        initializeArmMotor();
    }

    //movement control
    public void moveArm(double position){
        if(!enabled) return;
        desiredPosition = position;
        armMotor.setControl(mmReq.withPosition(desiredPosition).withFeedForward(arbitraryFeedForward));
    }

    public void moveArm(double position, StateMachineCallback callback){
        scoreStateMachineCallback = callback;
        moveArm(position);
    }

    //initialize arm motor
    private void initializeArmMotor(){

        System.out.println("armSubsystem: Starting UP & Initializing arm motor!");

        armMotor = new TalonFX(0);
        TalonFXConfiguration config = new TalonFXConfiguration();

        armMotor.getConfigurator().apply(config);
        armMotor.setNeutralMode(NeutralModeValue.Brake);    //see if we can include this in the config

        //TODO: setup and apply current limits to config
        /* Configure current limits */
        MotionMagicConfigs mm = config.MotionMagic;
        mm.MotionMagicCruiseVelocity = 0; // 5 rotations per second cruise
        mm.MotionMagicAcceleration = 0; // Ta200ke approximately 0.5 seconds to reach max vel
        // Take approximately 0.2 seconds to reach max accel
        mm.MotionMagicJerk = 0;

        Slot0Configs slot0 = config.Slot0;
        slot0.kP = 0;
        slot0.kI = 0;
        slot0.kD = 0;
        slot0.kV = 0;
        slot0.kS = 0; // Approximately 0.25V to get the mechanism moving

        FeedbackConfigs fdb = config.Feedback;
        fdb.SensorToMechanismRatio = 1;


        //applies config to ArmMotor
        StatusCode status = StatusCode.StatusCodeNotInitialized;
               config.MotorOutput.Inverted = ArmConstants.armDirection;
        for (int i = 0; i < 5; ++i) {
            status = armMotor.getConfigurator().apply(config);
            if (status.isOK())
                break;
        }
        if (!status.isOK()) {
            System.out.println("Could not configure device. Error: " + status.toString());
        }

        armMotor.setPosition(0);
        armMotor.setNeutralMode(NeutralModeValue.Brake);    //see if we can include this in the config

    }

    public void periodic(){
        if (!enabled) return;

        if (isAtPosition(desiredPosition) && scoreStateMachineCallback != null){
            scoreStateMachineCallback.setInput(ScoreInput.ARM_DONE);
            scoreStateMachineCallback = null;
        }

        log();
    }

    public double getArmPosition(){
        if (!enabled) return 0;
        return armMotor.getPosition().getValueAsDouble();
    }

    public boolean isAtPosition(double position){
        double tolerance = 2;
        return Math.abs(getArmPosition() - position) < tolerance;
    }

    private void log(){
        SmartDashboard.putNumber("arm motor position", armMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("desired arm position", desiredPosition);
        SmartDashboard.putNumber("arm motor closedLoopError",
            armMotor.getClosedLoopError().getValueAsDouble());
        SmartDashboard.putNumber("arm motor closedLoopReference",
            armMotor.getClosedLoopReference().getValueAsDouble());
        SmartDashboard.putNumber("arm motor closedLoopOutput",
            armMotor.getClosedLoopOutput().getValueAsDouble());
        SmartDashboard.putNumber("arm motor statorCurrent",
            armMotor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("arbitrary feed forward",
            arbitraryFeedForward);
    }
}
