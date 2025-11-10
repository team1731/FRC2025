package frc.lib.frc1731.log;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.util.struct.StructSerializable;

/**
 * Class to easily log to AdvantageKit under a named folder
 */
public class SmartLogger {
    private String logFolder = "/SmartLogs/";

    public SmartLogger(String name) {
        this.logFolder = logFolder + name + "/";
    }

    /**
     * Logs a double to AK
     */
    public void log(String key, double value) {
        Logger.recordOutput(logFolder + key, value);
    }

    /**
     * Logs a boolean to AK
     */
    public void log(String key, boolean value) {
        Logger.recordOutput(logFolder + key, value);
    }

    /**
     * Logs a WPILib geometry class (Pose2d, ChassisSpeeds, etc.) to AK
     */
    public void log(String key, StructSerializable value) {
        Logger.recordOutput(logFolder + key, value);
    }

    /**
     * Logs a String to AK
     */
    public void log(String key, String value) {
        Logger.recordOutput(logFolder + key, value);
    }

    /**
     * Logs an enum value to AK
     */
    public <T extends Enum<T>> void log(String key, T value) {
        Logger.recordOutput(logFolder + key, value);
    }

    /**
     * Logs a double to AK if the indicated condition is true, otherwise logs a different value
     */
    public void logIf(String key, double valueIfTrue, double valueIfFalse, boolean condition) {
        double loggedValue = valueIfFalse;
        if (condition) {
            loggedValue = valueIfTrue;
        }
        
        Logger.recordOutput(logFolder + key, loggedValue); // Record to AdvantageKit logs
    }

    /**
     * Logs a boolean to AK if the indicated condition is true, otherwise logs a different value
     */
    public void logIf(String key, boolean valueIfTrue, boolean valueIfFalse, boolean condition) {
        boolean loggedValue = valueIfFalse;
        if (condition) {
            loggedValue = valueIfTrue;
        }
        
        Logger.recordOutput(logFolder + key, loggedValue); // Record to AdvantageKit logs
    }

    /**
     * Logs a String to AK if the indicated condition is true, otherwise logs a different value
     */
    public void logIf(String key, String valueIfTrue, String valueIfFalse, boolean condition) {
        String loggedValue = valueIfFalse;
        if (condition) {
            loggedValue = valueIfTrue;
        }
        
        Logger.recordOutput(logFolder + key, loggedValue); // Record to AdvantageKit logs
    }

    /**
     * Logs a WPILib geometry class (Pose2d, ChassisSpeeds, etc.) to AK if the indicated condition is true, otherwise logs a different value
     */
    public void logIf(String key, StructSerializable valueIfTrue, StructSerializable valueIfFalse, boolean condition) {
        StructSerializable loggedValue = valueIfFalse;
        if (condition) {
            loggedValue = valueIfTrue;
        }
        
        Logger.recordOutput(logFolder + key, loggedValue); // Record to AdvantageKit logs
    }

    /**
     * Logs an enum value to AK if the indicated condition is true, otherwise logs a different value
     */
    public <E extends Enum<E>> void logIf(String key, E valueIfTrue, E valueIfFalse, boolean condition) {
        E loggedValue = valueIfFalse;
        if (condition) {
            loggedValue = valueIfTrue;
        }
        
        Logger.recordOutput(logFolder + key, loggedValue); // Record to AdvantageKit logs
    }
}