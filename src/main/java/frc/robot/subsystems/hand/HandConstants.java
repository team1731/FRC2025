package frc.robot.subsystems.hand;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.*;

import frc.lib.PIDGains;
import frc.lib.hardware.motor.PortConfig;

public final class HandConstants {
    // Motor Configs
    public final static int clamperCanId = 40;
    public final static int clamperCancoderDeviceId = 16;
    public final static int intakeCanId = 38;

    public final static String canBus = "canivore2";

    public final static InvertedValue clamperMotorDirection = InvertedValue.CounterClockwise_Positive;
    public final static InvertedValue intakeMotorDirection = InvertedValue.CounterClockwise_Positive;

    public final static PortConfig clamperPortConfig = new PortConfig(canBus, clamperCanId, clamperMotorDirection);
    public final static PortConfig intakePortConfig = new PortConfig(canBus, intakeCanId, intakeMotorDirection);

    // Intake Configs
    public final static double intakeStatorCurrentLimit = 20.0;

    public final static HardwareLimitSwitchConfigs intakeLimitSwitchConfigs = new HardwareLimitSwitchConfigs()
        .withReverseLimitEnable(false)
        .withReverseLimitSource(ReverseLimitSourceValue.LimitSwitchPin)
        .withForwardLimitEnable(false)
        .withForwardLimitSource(ForwardLimitSourceValue.LimitSwitchPin);

    public final static VoltageConfigs intakeVoltageConfigs = new VoltageConfigs()
        .withPeakForwardVoltage(12)
        .withPeakReverseVoltage(-12);

    /* Voltage-based velocity requires a feed forward to account for the back-emf of the motor */
    public final static PIDGains intakeGains = new PIDGains()
        .setP(0.22) // An error of 1 rotation per second results in 2V output
        .setI(0.5) // An error of 1 rotation per second increases output by 0.5V every second
        .setD(0.0001) // A change of 1 rotation per second squared results in 0.01 volts output
        .setV(0.12); // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / Rotation per second

    // Clamper Constants
    public final static double clamperStatorCurrentLimit = 40.0;

    public final static CANcoderConfiguration CANCoderConfigs = new CANcoderConfiguration()
        .withMagnetSensor(
            new MagnetSensorConfigs()
            .withMagnetOffset(0.195556640625)
            .withAbsoluteSensorDiscontinuityPoint(0.5)
        );

    public final static MotionMagicConfigs mmConfigs = new MotionMagicConfigs() 
        .withMotionMagicCruiseVelocity(70) // should be more like 70.0/125.0; // 5 rotations per second cruise
        .withMotionMagicAcceleration(250) // should be more like 140.0/125.0; // Take approximately 0.5 seconds to reach max vel
        // Take approximately 0.2 seconds to reach max accel
        .withMotionMagicJerk(0);

    public final static PIDGains clamperGains = new PIDGains()
        .setP(150.0) // seems like this should be more like 612
        .setI(0.0)
        .setD(0.0078125)
        .setV(0.009375)
        .setS(0.02); // Approximately 0.25V to get the mechanism moving

    public final static FeedbackConfigs clamperFeedbackConfigs = new FeedbackConfigs()
        .withFeedbackRemoteSensorID(clamperCancoderDeviceId)
        .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
        .withRotorToSensorRatio(125)
        .withSensorToMechanismRatio(1);

    // Intake Constants
    public final static double intakeCoralVelocity = 5000/60;
    public final static double intakeAlgaeVelocity = 5000/60;
    public final static double releaseVelocity = -5000/60;
    public final static double intakeHoldOutput = 0.1;
    public final static double intakeStoppedThreshold = 5;
    public final static double defaultReleaseRuntime = 1.0;

    // Clamper Constants
    public final static double minClamperPosition = 0;
    public final static double maxClamperPosition = 0.22;

    public final static double clamperHomePosition = 0.0;
    public final static double clamperCoralPosition = 0.019; //0.018554 --> start of competition value
    public final static double clamperAlgaePosition = 0.05;
    public final static double clamperPluckAlgaePosition = 0.05;
    public final static double clamperReefIntakePosition = 0.11; // note: this should be wide for intaking algae from the reef
    public final static double clamperDumpCoralPosition = 0.2; // note: this should be wide for dumping coral onto the reef
    public final static double clamperHoldCoral = -0.03; //DutyCycleOut

    // Position tolerance thresholds
    public final static double clamperPositionTolerance = 0.002;
}
