package frc.robot.subsystems.climb;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.frc1731.Utils;
import frc.lib.frc1731.hardware.MotorIOTalonFX;
import frc.lib.frc1731.subsystem.SingleMotorServoSubsystem;
import frc.robot.Constants;

public class ClimbSubsystem extends SingleMotorServoSubsystem<MotorIOTalonFX> {
    private boolean isClimbing = false;

    public ClimbSubsystem(boolean enabled) {
        super(enabled, ClimbConstants.climbAtPositionThreshold);
    }

    @Override
    protected void initializeHardware() {
        this.leadMotor = new MotorIOTalonFX(ClimbConstants.climbPortConfig);
        this.leadMotor.withCANCoder(
            ClimbConstants.climbCancoderDeviceId, 
            Constants.CANBUS_NAME,
            ClimbConstants.cancoderConfig
        );

        this.leadMotor.withMotionProfile(70d, 250d, 0d);
        this.leadMotor.withStatorCurrentLimit(80d);
        this.leadMotor.withPIDGains(ClimbConstants.climbPIDGains);
        this.leadMotor.withFeedbackConfigs(ClimbConstants.feedbackConfigs);
        this.leadMotor.setNeutralMode(NeutralModeValue.Brake);

        this.leadMotor.applyConfigs();
    }

    @Override
    public void periodicTelemetry() {
        logger.log("Current Position", getRawRotations());
        logger.log("Target Position", getTargetRotations());
        logger.log("At Target Position", getTargetRotations());
        logger.log("Is Climbing", this.isClimbing);
    }

    public boolean isClimbing() {
        return isClimbing;    
    }

    private Command moveClimbCommand(double position) {
        return setPositionCommand(Utils.clamp(position, ClimbConstants.minClimbPosition, ClimbConstants.maxClimbPosition));
    }

    public Command moveToMaxPositionCommand() {
        return moveClimbCommand(ClimbConstants.maxClimbPosition)
        .withName("MoveMax");
    }

    public Command moveToMinPositionCommand() {
        return moveClimbCommand(ClimbConstants.minClimbPosition)
        .withName("MoveMin");
    }

    public Command readyCommand() {
        return moveClimbCommand(ClimbConstants.climbReadyPosition)
        .withName("Ready");
    }

    public Command stopCommand() {
        return runOnce(() -> leadMotor.brake())
        .withName("Stop");
    }

    public Command stowCommand() {
        return moveClimbCommand(ClimbConstants.climbStowPosition)
        .withName("Stow");
    }

    public Command climbCommand() {
        return moveClimbCommand(ClimbConstants.climbHomePosition)
        .withName("Home");
    }

    public Command setIsClimbingCommand(boolean isClimbing) {
        return runOnce(() -> {
            this.isClimbing = isClimbing;
        }).withName("SetIsClimbing");
    }
}