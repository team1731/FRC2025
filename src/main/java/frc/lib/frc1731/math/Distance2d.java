package frc.lib.frc1731.math;

import edu.wpi.first.math.util.Units;

/**
 * Class that normalizes and converts between different distance units
 */
public class Distance2d {
    private double distanceInMeters;

    public Distance2d(double distanceInMeters) {
        this.distanceInMeters = distanceInMeters;
    }

    public Distance2d() {
        this(0.0);
    }

    /**
     * Creates a distance2d object from a distance in meters
     */
    public static Distance2d fromMeters(double distanceInMeters) {
        return new Distance2d(distanceInMeters);
    }

    /**
     * Creates a distance2d object from a distance in feet
     */
    public static Distance2d fromFeet(double distanceInMeters) {
        return new Distance2d(Units.feetToMeters(distanceInMeters));
    }

    /**
     * Creates a distance2d object from a distance in inches
     */
    public static Distance2d fromInches(double distanceInMeters) {
        return new Distance2d(Units.inchesToMeters(distanceInMeters));
    }

    /**
     * Addition of current and other distance
     */
    public Distance2d plus(Distance2d other) {
        return Distance2d.fromMeters(distanceInMeters + other.toMeters());
    }

    /**
     * Subtraction of current and other distance
     */
    public Distance2d minus(Distance2d other) {
        return plus(Distance2d.fromMeters(-other.toMeters()));
    }

    /**
     * Multiplication of current and other distance
     */
    public Distance2d multiply(Distance2d other) {
        return Distance2d.fromMeters(distanceInMeters * other.toMeters());
    }

    /**
     * Division of current and other distance
     */
    public Distance2d divide(Distance2d other) {
        return multiply(Distance2d.fromMeters(1.0/other.toMeters()));
    }

    /**
     * Returns the distance in meters
     */
    public double toMeters() {
        return distanceInMeters;
    }

    /**
     * Returns the distance in feet
     */
    public double toFeet() {
        return Units.metersToFeet(distanceInMeters);
    }

    /**
     * Returns the distance in meters
     */
    public double toInches() {
        return Units.metersToInches(distanceInMeters);
    }

    /**
     * Returns a string describing the current object
     */
    public String toString() {
        return "distance (meters): " + distanceInMeters;
    }
}