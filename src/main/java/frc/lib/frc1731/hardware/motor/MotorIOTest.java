package frc.lib.frc1731.hardware.motor;

import frc.lib.frc1731.PIDGains;
import frc.lib.frc1731.hardware.motor.ctre.*;
import frc.lib.frc1731.hardware.motor.rev.*;
import frc.lib.frc1731.log.SmartLogger;
import edu.wpi.first.wpilibj2.command.*;

/**
 * Test class that treats a motor as a subsystem; useful for easy initial testing of motors and prototypes
 */
public class MotorIOTest extends SubsystemBase {
    protected OLD_MotorIO testMotor; // accesible to other test motors
    private double desiredOutput = 0d;
    private SmartLogger logger;
    private OutputType outputType = OutputType.kNone;

    private enum OutputType {
        kPercentOutput,
        kVelocity,
        kPosition,
        kNone
    }

    public MotorIOTest(PortConfig config, Class<? extends OLD_MotorIO> cls) {
        if (cls.getSimpleName().equals(OLD_MotorIOSparkFlex.class.getSimpleName())) {
            this.testMotor = new OLD_MotorIOSparkFlex(config);
        } else if (cls.getSimpleName().equals(OLD_MotorIOSparkMax.class.getSimpleName())) {
            this.testMotor = new OLD_MotorIOSparkMax(config);
        } else if (cls.getSimpleName().equals(OLD_MotorIOTalonFX.class.getSimpleName())) {
            this.testMotor = new OLD_MotorIOTalonFX(config);
        } else if (cls.getSimpleName().equals(OLD_MotorIOTalonFXS.class.getSimpleName())) {
            this.testMotor = new OLD_MotorIOTalonFXS(config);
        }

        this.logger = new SmartLogger("TestMotor/" + cls.getSimpleName() + "[" + config.kPort + "]");
        this.logger.log("Class Type", cls.getSimpleName());
    }

    /**
     * Adds an inverted follower motor to this test motor
     */
    public MotorIOTest withFollower(MotorIOTest other, boolean inverted) {
        other.testMotor.setFollowerTo(this.testMotor, inverted);
        return this;
    }

    /**
     * Adds a follower motor to this test motor
     */
    public MotorIOTest withFollower(MotorIOTest other) {
        return this.withFollower(other, false);
    }

    /**
     * Adds PID gains to this test motor
     */
    public MotorIOTest withGains(PIDGains profile) {
        this.testMotor.withGains(profile);
        return this;
    }

    /**
     * Get current position in rotations
     */
    public double getRotations() {
        return this.testMotor.getRotations();
    }

    /**
     * Get current velocity in RPM
     */
    public double getVelocityRPM() {
        return this.testMotor.getVelocityRPM();
    }

    /**
     * Get the voltage applied to the motor
     */
    public double getAppliedVoltage() {
        return this.testMotor.getAppliedVoltage();
    }

    /**
     * Set the desired output of the motor to a percentage of maximum output
     */
    public Command setPercentOutput(double percent) {
        return new InstantCommand(() -> {
            this.desiredOutput = percent;
            this.testMotor.setPercentOutput(percent);
            this.outputType = OutputType.kPercentOutput;
        });
    }

    /**
     * Set the velocity of the motor in RPM
     */
    public Command setVelocityRPM(double velocityRPM) {
        return new InstantCommand(() -> {
            this.desiredOutput = velocityRPM;
            this.testMotor.setVelocity(velocityRPM);
            this.outputType = OutputType.kVelocity;
        });
    }

    /**
     * Set the position of the motor in rotations
     */
    public Command setPosition(double rotations) {
        return new InstantCommand(() -> {
            this.desiredOutput = rotations;
            this.testMotor.setPosition(rotations);
            this.outputType = OutputType.kPosition;
        });
    }

    @Override
    public void periodic() {
        this.logger.log("Desired Output", desiredOutput);
        this.logger.log("Output Type", outputType);
    }
}
