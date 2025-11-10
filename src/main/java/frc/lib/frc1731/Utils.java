package frc.lib.frc1731;

/**
 * General utility class with helper methods
 */
public final class Utils {
    /**
     * Clamps the value between a given range
     */
    public static double clamp(double value, double min, double max) {
        return Math.max(Math.min(value, max), min);
    }

    /**
     * Clamps the value between a given range
     */
    public static double clamp(double value, double range) {
        return Math.max(Math.min(value, range), -range);
    }

    /**
     * Rounds the number to a certain number of decimal places
     */
    public static double roundTo(double value, double decimalPlaces) {
        return Math.round(value*Math.pow(10, decimalPlaces))/Math.pow(10, decimalPlaces);
    }

    /**
     * Whether the particlar value is within a specified tolerance of the target
     */
    public static boolean isWithin(double value, double target, double tolerance) {
        return Math.abs(target - value) <= tolerance;
    }
}