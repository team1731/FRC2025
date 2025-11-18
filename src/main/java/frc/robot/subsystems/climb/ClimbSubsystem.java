package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.frc1731.hardware.MotorIOTalonFX;
import frc.lib.frc1731.subsystem.PivotMotorSubsystem;
import frc.robot.Constants;

public class ClimbSubsystem extends PivotMotorSubsystem<MotorIOTalonFX> {
    private boolean isClimbing = false;

    public ClimbSubsystem(boolean enabled) {
        super(enabled);
        super.setMotorTolerance(ClimbConstants.climbAtPositionThreshold);
        super.withSimulation(
            ClimbConstants.simConstants, 
            ClimbConstants.climbPIDGains
        );
    }

    @Override
    protected void configureHardware() {
        this.motor = new MotorIOTalonFX(ClimbConstants.climbPortConfig);
        this.motor.withCANCoder(
            ClimbConstants.climbCancoderDeviceId, 
            Constants.CANBUS_NAME,
            ClimbConstants.cancoderConfig
        );

        this.motor.withMotionProfile(70d, 250d, 0d);
        this.motor.withStatorCurrentLimit(80d);
        this.motor.withPIDGains(ClimbConstants.climbPIDGains);
        this.motor.withFeedbackConfigs(ClimbConstants.feedbackConfigs);
        this.motor.setNeutralMode(NeutralModeValue.Brake);

        this.motor.applyConfigs();
    }

    @Override
    public void periodicTelemetry() {
        logger.log("Current Position", getPosition().in(Rotations));
        logger.log("Target Position", getTargetPosition().in(Rotations));
        logger.log("At Target Position", atTargetPosition());
        logger.log("Is Climbing", this.isClimbing);
    }

    public boolean isClimbing() {
        return isClimbing;    
    }

    private Command moveClimbCommand(Angle position) {
        return setPositionCommand(position);
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
        return runOnce(() -> motor.brake())
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