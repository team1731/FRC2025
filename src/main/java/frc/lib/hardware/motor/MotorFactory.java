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
    private static List<OLD_MotorIO> motors = new ArrayList<>();

    public static OLD_MotorIOTalonFX createDefaultTalonFX(PortConfig config) {
        int port = config.kPort;
        if (motors.get(port) == null) {
            motors.add(port, new OLD_MotorIOTalonFX(config));
        }

        return (OLD_MotorIOTalonFX) motors.get(port);
    }

    public static OLD_MotorIOSparkMax createDefaultSparkMax(PortConfig config) {
        int port = config.kPort;
        if (motors.get(port) == null) {
            motors.add(port, new OLD_MotorIOSparkMax(config));
        }

        return (OLD_MotorIOSparkMax) motors.get(port);
    }

    public static OLD_MotorIOSparkFlex createDefaultSparkFlex(PortConfig config) {
        int port = config.kPort;
        if (motors.get(port) == null) {
            motors.add(port, new OLD_MotorIOSparkFlex(config));
        }

        return (OLD_MotorIOSparkFlex) motors.get(port);
    }

    public static OLD_MotorIOTalonFXS createDefaultTalonFXS(PortConfig config) {
        int port = config.kPort;
        if (motors.get(port) == null) {
            motors.add(port, new OLD_MotorIOTalonFXS(config));
        }
        
        return (OLD_MotorIOTalonFXS) motors.get(port);
    }

    public static <M extends OLD_MotorIO> boolean isOfMotorType(int port, Class<M> cls) {
        return motors.get(port) != null && motors.get(port).getClass().equals(cls);
    }
}
