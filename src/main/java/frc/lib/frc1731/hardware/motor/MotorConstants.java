package frc.lib.frc1731.hardware.motor;

/**
 * Helper class that holds hardware constants for different motors
 */
public enum MotorConstants {
    KRAKEN_X60(6000d, 366d, 7.09, 502.1),
    KRAKEN_X44(7530d, 275d, 4.05, 630.7),
    VORTEX(6784d, 211d, 3.6, 575.1),
    NEO(5676d, 105d, 2.6, 493.5),
    FALCON_500(6380d, 257d, 4.69, 534.8);
    ;
    public final double MAX_VELOCITY_RPM;
    public final double STALL_CURRENT_AMPS;
    public final double STALL_TORQUE_NM;
    public final double MOTOR_KV;

    private MotorConstants(double maxRPM, double stallAmp, double stallTorque, double motorKV) {
        this.MAX_VELOCITY_RPM = maxRPM;
        this.STALL_CURRENT_AMPS = stallAmp;
        this.STALL_TORQUE_NM = stallTorque;
        this.MOTOR_KV = motorKV;
    }
}