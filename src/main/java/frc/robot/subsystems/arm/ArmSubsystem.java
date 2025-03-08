package frc.robot.subsystems.arm;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.state.StateMachineCallback;
import frc.robot.state.sequencer.SequenceInput;
import frc.robot.subsystems.ToggleableSubsystem;
import frc.robot.subsystems.climb.ClimbConstants;
import frc.robot.subsystems.climb.ClimbSubsystem;


public class ArmSubsystem extends SubsystemBase implements ToggleableSubsystem{
    
    
    private TalonFX armMotor;
    private CANcoder armCANcoder;
    private DynamicMotionMagicVoltage mmReq = new DynamicMotionMagicVoltage(
        0, ArmConstants.normalArmVelocity, ArmConstants.normalArmAcceleration, ArmConstants.armJerk);
    private final NeutralOut brake = new NeutralOut();

    private double desiredPosition;
    private boolean enabled;
    private double arbitraryFeedForward = 0;

    // state machine callback handling
    private StateMachineCallback stateMachineCallback;
    private boolean callbackOnDone = false;
    private boolean callbackOnThreshold = false;
    private double positionThreshold = 0;
    private boolean forwardThreshold = false;

    private ClimbSubsystem climbSubsystem;


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

    public void setClimbSubsystem(ClimbSubsystem climbSubsystem) {
        this.climbSubsystem = climbSubsystem;
    }

    //movement control
    private void moveArm(double position){
        if(!enabled) return;

        //check if climber will collide with regular arm movments
        if(climbSubsystem.getClimbPosition() > ClimbConstants.climbArmStowThreshold){
            position = ArmConstants.stowArmPosition; // override any incoming positon with the stow position
        }

        position = position * ArmConstants.armGearRationModifier;
        // do not go outside boundary thresholds
        if(position > ArmConstants.maxArmPosition) {
            desiredPosition = ArmConstants.maxArmPosition;
        } else if(position < ArmConstants.minArmPosition) {
            desiredPosition = ArmConstants.minArmPosition;
        } else {
            desiredPosition = position;
        }

        armMotor.setControl(mmReq.withPosition(desiredPosition).withFeedForward(arbitraryFeedForward));
    }

    public void moveArmNormalSpeed(double position) {
        mmReq.Velocity = ArmConstants.normalArmVelocity;
        mmReq.Acceleration = ArmConstants.normalArmAcceleration;
        System.out.println("ArmSubsystem normal velocity: " + ArmConstants.normalArmVelocity + " accel: " + ArmConstants.normalArmAcceleration);
        callbackOnDone = true;
        moveArm(position);
    }

    public void moveArmNormalSpeed(double position, StateMachineCallback callback){
        stateMachineCallback = callback;
        moveArmNormalSpeed(position);
    }

    public void moveArmNormalSpeed(double position, StateMachineCallback callback, double threshold){
        stateMachineCallback = callback;
        callbackOnDone = true;
        callbackOnThreshold = true;
        positionThreshold = threshold * ArmConstants.armGearRationModifier;
        forwardThreshold = threshold < position;
        moveArmNormalSpeed(position);
    }

    public void moveArmSlowSpeed(double position) {
        mmReq.Velocity = ArmConstants.slowedArmVelocity;
        mmReq.Acceleration = ArmConstants.slowedArmAcceleration;
        System.out.println("ArmSubsystem slowed velocity: " + ArmConstants.slowedArmVelocity + " accel: " + ArmConstants.slowedArmAcceleration);
        callbackOnDone = true;
        moveArm(position);
    }

    public void moveArmSlowSpeed(double position, StateMachineCallback callback){
        stateMachineCallback = callback;
        moveArmSlowSpeed(position);
    }

    public void stopArm() {
        if(!enabled) return;
        armMotor.setControl(brake);
    }

    //initialize arm motor
    private void initializeArmMotor(){

        System.out.println("armSubsystem: Starting UP & Initializing arm motor!");

        armCANcoder = new CANcoder(ArmConstants.armCancoderDeviceId, "canivore1");
        CANcoderConfiguration cancoderConfigs = new CANcoderConfiguration();
        cancoderConfigs.MagnetSensor.MagnetOffset = -0.217529296875;     //-0.216552734375
        cancoderConfigs.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5; // TODO what should this be?
        cancoderConfigs.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        armCANcoder.getConfigurator().apply(cancoderConfigs);

        armMotor = new TalonFX(ArmConstants.armCanId, "canivore1");
        TalonFXConfiguration config = new TalonFXConfiguration();

        armMotor.getConfigurator().apply(config);
        
        /* Configure current limits */
        MotionMagicConfigs mm = config.MotionMagic;
        mm.MotionMagicCruiseVelocity = ArmConstants.normalArmVelocity; 
        mm.MotionMagicAcceleration = ArmConstants.normalArmAcceleration; 
        mm.MotionMagicJerk = ArmConstants.armJerk;

        Slot0Configs slot0 = config.Slot0;
        slot0.kP = 90;
        slot0.kI = 0;
        slot0.kD = 0.0099;
        slot0.kV = 0.9;
        slot0.kS = 0.02; // Approximately 0.25V to get the mechanism moving

        FeedbackConfigs fdb = config.Feedback;
        fdb.SensorToMechanismRatio = 1;
        fdb.FeedbackRemoteSensorID = armCANcoder.getDeviceID();;
        fdb.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        fdb.RotorToSensorRatio = 1600.0/18.0;
        config.CurrentLimits.StatorCurrentLimit = 40;
        config.CurrentLimits.StatorCurrentLimitEnable = true;


        //applies config to ArmMotor
        StatusCode status = StatusCode.StatusCodeNotInitialized;
               config.MotorOutput.Inverted = ArmConstants.armMotorDirection;
        for (int i = 0; i < 5; ++i) {
            status = armMotor.getConfigurator().apply(config);
            if (status.isOK())
                break;
        }
        if (!status.isOK()) {
            System.out.println("Could not configure device. Error: " + status.toString());
        }

        armMotor.setPosition(0);
        armMotor.setNeutralMode(NeutralModeValue.Brake);

    }

    public void periodic(){
        if (!enabled) return;

        if(climbSubsystem.isClimbing() && climbSubsystem.getClimbPosition() > ClimbConstants.climbArmStowThreshold){
             moveArm(ArmConstants.stowArmPosition);
        }

        /*
         * Score State Machine callback handling
         */
        if(callbackOnDone && isAtPosition(desiredPosition) && stateMachineCallback != null) {
            // final position reached, notify the state machine
            callbackOnDone = false;
            System.out.println("Arm subsystem callback: " + getArmPosition());
            stateMachineCallback.setInput(SequenceInput.ARM_DONE);
        } else if(callbackOnThreshold && stateMachineCallback != null) {
            // check to see if the threshold was met, if so notify the state machine
            boolean thresholdMet = forwardThreshold && getArmPosition() >= positionThreshold ||
                !forwardThreshold && getArmPosition() <= positionThreshold;
            if(thresholdMet) {
                System.out.println("Arm subsystem threshold callback: " + getArmPosition());
                stateMachineCallback.setInput(SequenceInput.ARM_THRESHOLD_MET);
                callbackOnThreshold = false;
                positionThreshold = 0;
            }
        }

        log();
    }

    public double getArmPosition(){
        if (!enabled) return 0;
        return armMotor.getPosition().getValueAsDouble();
    }

    public boolean isAtPosition(double position){
        double tolerance = .06;
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
