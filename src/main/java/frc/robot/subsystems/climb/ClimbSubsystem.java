package frc.robot.subsystems.climb;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.ToggleableSubsystem;


public class ClimbSubsystem extends SubsystemBase implements ToggleableSubsystem{
        
    private TalonFX climbMotor;
    private CANcoder climbCancoder;
    private MotionMagicVoltage mmReq = new MotionMagicVoltage(0);
    private final NeutralOut brake = new NeutralOut();

    private double desiredPosition;
    private boolean enabled;
    private double arbitraryFeedForward = 0;
    

    @Override
    public boolean isEnabled() {
        return enabled;
    }

    public ClimbSubsystem(boolean enabled) {
        this.enabled = enabled;
        if (!enabled)
            return;
        initializeClimbMotor();
    }
    
    public void moveClimb(double position){

        // do not go outside boundary thresholds
        if(position > ClimbConstants.maxClimbPosition) {
            desiredPosition = ClimbConstants.maxClimbPosition;
        } else if(position < ClimbConstants.minClimbPosition) {
            desiredPosition = ClimbConstants.minClimbPosition;
        } else {
            desiredPosition = position;
        }

        climbMotor.setControl(mmReq.withPosition(desiredPosition).withFeedForward(arbitraryFeedForward));
    }

    public void stopClimb(){
        if(!enabled) return;
        climbMotor.setControl(brake);
    }

    public void stowClimb(){
        if(!enabled) return;
        moveClimb(ClimbConstants.climbStowedPosition);
    }

    public void initializeClimbMotor(){
        if (!enabled) return;
        
        System.out.println("ClimbSubsystem: Starting Up & Initializing Climb Motor !!!!");

        climbCancoder = new CANcoder(ClimbConstants.climbCancoderDeviceId, "canivore1");
        CANcoderConfiguration cancoderConfig = new CANcoderConfiguration();
        cancoderConfig.MagnetSensor.MagnetOffset = -0.179931640625; 
        cancoderConfig.MagnetSensor.SensorDirection = ClimbConstants.climbCanConderDirection;
        cancoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.900146484375;
        climbCancoder.getConfigurator().apply(cancoderConfig);

        climbMotor = new TalonFX(ClimbConstants.climbCanId, "canivore1");
        TalonFXConfiguration config = new TalonFXConfiguration();
        climbMotor.getConfigurator().apply(config);
        
         /* Configure current limits */
        MotionMagicConfigs mm = config.MotionMagic;
        mm.MotionMagicCruiseVelocity = 70; // 5 rotations per second cruise
        mm.MotionMagicAcceleration = 250; // Ta200ke approximately 0.5 seconds to reach max vel
        // Take approximately 0.2 seconds to reach max accel
        mm.MotionMagicJerk = 0;

        Slot0Configs slot0 = config.Slot0;
        slot0.kP = 240;
        slot0.kI = 0;
        slot0.kD = 0.0078125;
        slot0.kV = 0.009375; 
        slot0.kS = 0.02; // Approximately 0.25V to get the mechanism moving

        FeedbackConfigs fdb = config.Feedback;
        fdb.FeedbackRemoteSensorID = climbCancoder.getDeviceID();
        fdb.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        fdb.RotorToSensorRatio = 800/1; 
        fdb.SensorToMechanismRatio = 1;
        //for testing
        config.CurrentLimits.StatorCurrentLimit = 20;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

         // Apply the configs to Motor 
        config.MotorOutput.Inverted = ClimbConstants.climbMotorDirection;
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = climbMotor.getConfigurator().apply(config);
            if (status.isOK())
                break;
        }
        if (!status.isOK()) {
            System.out.println("Could not configure device. Error: " + status.toString());
        }

        climbMotor.setPosition(0);
        climbMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    public void periodic(){
        if (!enabled) return;
        
        log();
    }

    public double getClimbPosition(){
        if (!enabled) 
            return 0;
        return climbMotor.getPosition().getValueAsDouble();
    }

    public boolean isAtPosition(double position){
        double tolerance = 2;
        return Math.abs(getClimbPosition() - position) < tolerance;
    }

    public void log(){
                SmartDashboard.putNumber("climb motor position", climbMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("desired climb position", desiredPosition);
        SmartDashboard.putNumber("climb motor closedLoopError",
            climbMotor.getClosedLoopError().getValueAsDouble());
        SmartDashboard.putNumber("climb motor closedLoopReference",
            climbMotor.getClosedLoopReference().getValueAsDouble());
        SmartDashboard.putNumber("climb motor closedLoopOutput",
            climbMotor.getClosedLoopOutput().getValueAsDouble());
        SmartDashboard.putNumber("climb motor statorCurrent",
            climbMotor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("arbitrary feed forward",
            arbitraryFeedForward);
    }
}
