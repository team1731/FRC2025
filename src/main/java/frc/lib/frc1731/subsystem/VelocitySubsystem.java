package frc.lib.frc1731.subsystem;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.frc1678.sim.RollerSim;
import frc.lib.frc1678.sim.RollerSim.RollerSimConstants;
import frc.lib.frc1731.PIDGains;
import frc.lib.frc1731.hardware.motor.MotorIO;
import frc.robot.Robot;

public abstract class VelocitySubsystem<M extends MotorIO> extends BaseSubsystem {
    protected M motor; // The primary motor controller

    private AngularVelocity targetVelocity = RotationsPerSecond.zero();

    private RollerSim sim = null; // The simulated model of the subsystem
    private PIDGains simPIDGains; // The active PIDController for simulating the subsystem
    private SimpleMotorFeedforward simFF; // The feedforward component for the sim PID controller

    private double mechanismRatio = 1d; // Mechanism input:output reduction ratio (i.e 2:1 ratio means 2 motor rotations per mechanism rotation)

    protected VelocitySubsystem(boolean enabled) {
        super(enabled);
        if (!isEnabled()) return;
        initializeHardware();
    }

    protected abstract void initializeHardware();

    @Override
    public void periodic() {
        super.periodic();
        sim.simulate();
    }

    protected void withSimulation(RollerSimConstants constants, PIDGains simGains) {
        this.sim = new RollerSim(constants);
        this.simPIDGains = simGains;
        this.simFF = new SimpleMotorFeedforward(
            simGains.kS,
            simGains.kV,
            simGains.kA
        );
    }

    protected void setVelocity(AngularVelocity velocity) {
        this.setVelocity(velocity, 0);
    }

    protected void setVelocity(AngularVelocity velocity, int pidSlot) {
        this.targetVelocity = velocity.times(mechanismRatio);
        this.motor.setVelocityRPS(targetVelocity.in(RotationsPerSecond), pidSlot);

        if (sim == null || simPIDGains == null) return;
        double curVelocity = getVelocity().in(RotationsPerSecond);
        double tarVelocity = targetVelocity.in(RotationsPerSecond);

        double desiredVoltage = simPIDGains.toPIDController().calculate(curVelocity, tarVelocity) + simFF.calculate(curVelocity);
        sim.setVoltage(Volts.of(desiredVoltage));
    }

    protected void setPercentOutput(double targetPercent) {
        this.targetVelocity = RotationsPerSecond.of(targetPercent * 6380d / 60d); // TODO - Adjust max velocity based on motor
        this.motor.setPercentOutput(targetPercent);

        if (sim == null) return;
        this.sim.setVoltage(Volts.of(targetPercent * RobotController.getBatteryVoltage()));
    }

    protected void setVoltage(double targetVoltage) {
        this.targetVelocity = RotationsPerSecond.of(targetVoltage * 531.67 / 60d); // TODO - Adjust Kv based on motor
        this.motor.setVoltage(targetVoltage);

        if (sim == null) return;
        this.sim.setVoltage(Volts.of(targetVoltage));
    }

    protected AngularVelocity getVelocity() {
        return Robot.isSimulation() ? sim.getVelocity():
        RotationsPerSecond.of(motor.getVelocityRPS());
    }

    protected double getVelocityRPS() {
        return getVelocity().in(RotationsPerSecond);
    }

    protected AngularVelocity getTargetVelocity() {
        return this.targetVelocity;
    }

    protected double getTargetVelocityRPS() {
        return getTargetVelocity().in(RotationsPerSecond);
    }

    protected Command setVelocityCommand(AngularVelocity velocity) {
        return this.run(() -> this.setVelocity(velocity));
    }

    protected Command setVelocityCommand(double velocityRPS) {
        return this.setVelocityCommand(RotationsPerSecond.of(velocityRPS));
    }

    protected Command setPercentOutputCommand(double desiredPercent) {
        return this.run(() -> this.setPercentOutput(desiredPercent));
    }

    protected Command setVoltageCommand(double desiredVoltage) {
        return this.run(() -> this.setVoltage(desiredVoltage));
    }

    protected Command stopCommand() {
        return this.runOnce(() -> {
            this.motor.brake();
            this.targetVelocity = RotationsPerSecond.zero();
            this.sim.setVoltage(Volts.of(0d));
        });
    }
}