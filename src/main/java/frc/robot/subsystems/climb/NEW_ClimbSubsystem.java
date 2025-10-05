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

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.Utils;
import frc.robot.subsystems.ToggleableSubsystem;

public class NEW_ClimbSubsystem extends SubsystemBase implements ToggleableSubsystem {
    private boolean isEnabled = false;
    private TalonFX climbMotor;
    private CANcoder climbCancoder;
    private MotionMagicVoltage mmReq = new MotionMagicVoltage(0);

    private double targetPosition = 0.0;
    private boolean isClimbing = false;

    public NEW_ClimbSubsystem(boolean enabled) {
        this.isEnabled = enabled;
        if (!isEnabled) return;

        climbCancoder = new CANcoder(ClimbConstants.climbCancoderDeviceId, "canivore1");
        CANcoderConfiguration cancoderConfig = new CANcoderConfiguration();
        cancoderConfig.MagnetSensor.MagnetOffset = -0.315185546875; 
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
        fdb.RotorToSensorRatio = 640; 
        fdb.SensorToMechanismRatio = 1;
        
        //for testing
        config.CurrentLimits.StatorCurrentLimit = 80;
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

    @Override
    public boolean isEnabled() {
        return isEnabled;
    }

    @Override
    public void periodic() {
        if(isClimbing && targetPosition < ClimbConstants.climbResetThreshold && getClimbPosition() < ClimbConstants.climbResetThreshold) {
            // start button was pressed which set isClimbing to true
            // but now the driver is moving the climb back toward home and it's almost there
            // assume this is an attempt to reset
            isClimbing = false;
        }
    }

    public double getClimbPosition(){
        return isEnabled ? climbMotor.getPosition().getValueAsDouble() : 0;
    }

    public boolean isAtPosition(double position){
        double tolerance = 2;
        return Math.abs(getClimbPosition() - position) < tolerance;
    }

    public boolean isAtTargetPosition() {
        return isAtPosition(targetPosition);
    }

    public boolean isClimbing() {
        return isClimbing;
    }

    public void stowClimb(){
        if(!isEnabled) return;
        moveClimb(ClimbConstants.climbHomePosition);
    }

    private void moveClimb(double desiredPosition) {
        if (!isEnabled) return;
        double appliedPosition = Utils.clamp(desiredPosition, ClimbConstants.minClimbPosition, ClimbConstants.maxClimbPosition);
        this.targetPosition = appliedPosition;
        this.climbMotor.setControl(mmReq.withPosition(appliedPosition));
    }

    public Command stopCommand() {
        return runOnce(() -> climbMotor.setControl(new NeutralOut()))
        .onlyIf(() -> isEnabled)
        .withName("Stop");
    }

    public Command moveClimbCommand(double desiredPosition) {
        return run(() -> moveClimb(desiredPosition))
        .until(() -> isAtTargetPosition())
        .withName("MoveClimb");
    }

    public Command setIsClimbing(boolean climbing) {
        return runOnce(() -> this.isClimbing = climbing)
        .onlyIf(() -> isEnabled)
        .withName("SetIsClimbing");
    }
}