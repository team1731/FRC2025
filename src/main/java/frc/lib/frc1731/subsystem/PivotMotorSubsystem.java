package frc.lib.frc1731.subsystem;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.frc1678.sim.PivotSim;
import frc.lib.frc1678.sim.PivotSim.PivotSimConstants;
import frc.lib.frc1731.PIDGains;
import frc.lib.frc1731.hardware.motor.MotorIO;
import frc.robot.Robot;

public abstract class PivotMotorSubsystem<M extends MotorIO> extends BaseSubsystem {
    protected M motor = null; // The primary motor controller
    private PivotSim sim = null; // The simulated model of the subsystem 
    private PIDController simPID = null; // The active PIDController for simulating the subsystem
    private SimpleMotorFeedforward simFF = null; // The feedforward component for the sim PID controller

    private Angle epsilon = Degrees.zero(); // Tolerance for position control
    
    private Angle targetPosition = Degrees.zero();

    private double gearRatioScalar = 1.0; // Gear ratio reduction (i.e 3:1 means 3 input rotations per output rotation)

    public PivotMotorSubsystem(boolean enabled) {
        super(enabled);
    }

    protected abstract void configureHardware(); // Initilizes and configures all motors/sensors

    protected void withSimulation(PivotSimConstants constants, PIDGains simGains) {
        this.sim = new PivotSim(constants);
        this.simPID = simGains.toPIDController();
        this.simFF = new SimpleMotorFeedforward(
            simGains.kS,
            simGains.kV,
            simGains.kA
        );
    }

    protected void withGearRatio(double gearRatio) {
        this.gearRatioScalar = gearRatio;
    }

    @Override
    public void periodic() {
        super.periodic();
        if (sim != null) {
            sim.simulate();
        }
    }

    protected Angle getTolerance() {
        return epsilon;
    }

    protected Angle toMechanism(Angle motorAngle) {
        return motorAngle.times(gearRatioScalar);
    }

    protected Angle toMotor(Angle mechanismAngle) {
        return mechanismAngle.div(gearRatioScalar);
    }

    protected void setTolerance(Angle epsilon) {
        this.epsilon = epsilon;
    }

    protected void setMotorTolerance(Angle epsilon) {
        this.epsilon = toMechanism(epsilon);
    }

    protected void setTolerence(double epsilon) {
        this.setTolerance(Degrees.of(epsilon));
    }

    protected void setMotorTolerence(double epsilon) {
        this.setMotorTolerance(Rotations.of(epsilon));
    }

    public Angle getTargetPosition() {
        return targetPosition;
    }

    public Angle getTargetMotorPosition() {
        return toMotor(targetPosition);
    }

    public Angle getPosition() {
        return Robot.isSimulation() ? sim.getPosition() : 
            toMechanism(Rotations.of(motor.getRotations()));
    }

    public Angle getMotorPosition() {
        return Robot.isSimulation() ? sim.getPosition() : Rotations.of(motor.getRotations());
    }

    public AngularVelocity getVelocity() {
        return Robot.isSimulation() ? sim.getVelocity() : 
            toMechanism(Rotations.of(motor.getVelocityRPS())).per(Second);
    }

    protected void setMotorRotations(Angle rotations, int pidSlot) {
        motor.setPosition(rotations.in(Rotation), pidSlot);
    }

    protected void setMotorRotations(Angle rotations) {
        this.setMotorRotations(rotations, 0);
    }

    protected void setMotorRotations(double rotations, int pidSlot) {
        this.setMotorRotations(Rotations.of(rotations), pidSlot);
    }

    protected void setMotorRotations(double rotations) {
        this.setMotorRotations(Rotations.of(rotations), 0);
    }

    protected void setPosition(Angle position, int pidSlot) {
        this.targetPosition = position;
        Angle targetMotorPosition = toMotor(position);
        this.setMotorRotations(targetMotorPosition, pidSlot);

        if (sim == null || simPID == null) {
            double desiredVoltage = simPID.calculate(getMotorPosition().in(Rotations), targetMotorPosition.in(Rotations))
                + simFF.calculate(getVelocity().in(RotationsPerSecond));

            this.sim.setVoltage(Volts.of(desiredVoltage));
        }
    }

    protected void setPosition(Angle position) {
        this.setPosition(position, 0);
    }

    protected void setPosition(double position, int pidSlot) {
        this.setPosition(Degrees.of(position), pidSlot);
    }

    protected void setPosition(double position) {
        this.setPosition(position, 0);
    }

    protected boolean atTargetPosition() {
        return getPosition().isNear(targetPosition, epsilon);
    }

    protected boolean atTargetPosition(Angle epsilon) {
        return getPosition().isNear(targetPosition, epsilon);
    }

    protected boolean atPosition(Angle desiredPosition) {
        return getPosition().isNear(desiredPosition, epsilon);
    }

    protected boolean atPosition(Angle desiredPosition, Angle epsilon) {
        return getPosition().isNear(desiredPosition, epsilon);
    }

    protected Command setMotionProfileSpeeds(double velocity, double acceleration) {
        return new InstantCommand(() -> {
            motor.withMotionProfile(velocity, acceleration, 0d);
        });
    }

    protected Command setPositionCommand(Angle position) {
        return run(() -> setPosition(position))
        .until(() -> atPosition(position));
    }

    protected Command setPositionCommand(Angle position, Angle epsilon) {
        return run(() -> setPosition(position))
        .until(() -> atPosition(position, epsilon));
    }

    protected Command stopCommand() {
        return this.runOnce(() -> {
            motor.brake();
            if (sim != null) sim.setVoltage(Volts.of(0));
        });
    }
}