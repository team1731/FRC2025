package frc.lib.hardware.motor;

import java.util.ArrayList;
import java.util.List;

import frc.lib.hardware.motor.ctre.*;
import frc.lib.hardware.motor.rev.*;

/**
 * Utility class for creating and managing motors
 * 
 * TODO - Probably needs testing or fixing but I'm too lazy now to do it
 */
public class MotorFactory {
    private static List<MotorIO> motors = new ArrayList<>();

    public static MotorIOTalonFX createDefaultTalonFX(PortConfig config) {
        int port = config.kPort;
        if (motors.get(port) == null) {
            motors.add(port, new MotorIOTalonFX(config));
        }

        return (MotorIOTalonFX) motors.get(port);
    }

    public static MotorIOSparkMax createDefaultSparkMax(PortConfig config) {
        int port = config.kPort;
        if (motors.get(port) == null) {
            motors.add(port, new MotorIOSparkMax(config));
        }

        return (MotorIOSparkMax) motors.get(port);
    }

    public static MotorIOSparkFlex createDefaultSparkFlex(PortConfig config) {
        int port = config.kPort;
        if (motors.get(port) == null) {
            motors.add(port, new MotorIOSparkFlex(config));
        }

        return (MotorIOSparkFlex) motors.get(port);
    }

    public static MotorIOTalonFXS createDefaultTalonFXS(PortConfig config) {
        int port = config.kPort;
        if (motors.get(port) == null) {
            motors.add(port, new MotorIOTalonFXS(config));
        }
        
        return (MotorIOTalonFXS) motors.get(port);
    }

    public static <M extends MotorIO> boolean isOfMotorType(int port, Class<M> cls) {
        return motors.get(port) != null && motors.get(port).getClass().equals(cls);
    }
}
