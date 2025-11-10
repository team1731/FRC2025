package frc.lib.frc1731.math;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * Class that represents a coordinate in a 2d plane
 */
public class Coordinate2d {
    private Translation2d translationMeters;

    public Coordinate2d(Distance2d x, Distance2d y) {
        this.translationMeters = new Translation2d(x.toMeters(), y.toMeters());
    }

    public Coordinate2d(Translation2d translation) {
        this.translationMeters = translation;
    }

    /**
     * Mirrors the coordinate across a particular x plane
     */
    public Coordinate2d mirrorFromX(double x) {
        double newX = 2*x - translationMeters.getX();
        translationMeters = new Translation2d(newX, translationMeters.getY());
        return this;
    }

    /**
     * Mirrors the coordinate across a particular y plane
     */
    public Coordinate2d mirrorFromY(double y) {
        double newY = 2*y - translationMeters.getY();
        translationMeters = new Translation2d(translationMeters.getX(), newY);
        return this;
    }

    /**
     * Mirrors the coordinate around another coordinate
     */
    public Coordinate2d mirrorFromCoordinate(Coordinate2d coordinate) {
        double newX = 2*coordinate.getX().toMeters() - translationMeters.getX();
        double newY = 2*coordinate.getY().toMeters() - translationMeters.getY();
        translationMeters = new Translation2d(newX, newY);
        return this;
    }

    /**
     * Returns the x coordinate in meters
     */
    public Distance2d getX() {
        return Distance2d.fromMeters(translationMeters.getX());
    }

    /**
     * Returns the t coordinate in meters
     */
    public Distance2d getY() {
        return Distance2d.fromMeters(translationMeters.getX());
    }

    /**
     * Returns the x, y coordinate as a {@code Translation2d} object
     */
    public Translation2d getTranslation() {
        return translationMeters;
    }

    /**
     * Returns the distance between this coordinate and the indicated coordinate
     */
    public Distance2d getDistanceFromCoordinate(Coordinate2d other) {
        return Distance2d.fromMeters(translationMeters.getDistance(other.getTranslation()));
    }

    /**
     * Returns a vector between this coordinate and the indicated coordinate
     */
    public Vector2d vectorBetweenCoordinates(Coordinate2d other) {
        return new Vector2d(
            other.getX().minus(this.getX()),
            other.getY().minus(this.getY())
        );
    }

    /**
     * Converts this coordinate to a string representation
     */
    public String toString() {
        return "(" + translationMeters.getX() + ", " + translationMeters.getY() + ")";
    }
}