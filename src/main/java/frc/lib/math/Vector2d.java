package frc.lib.math;


import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Class that represents a vector in 2d space
 */
public class Vector2d {
    private Distance2d magnitude;
    private Rotation2d direction;

    public Vector2d() {
        this.magnitude = new Distance2d();
        this.direction = new Rotation2d();
    }

    public Vector2d(Distance2d magnitude, Rotation2d direction) {
        this.magnitude = magnitude;
        this.direction = direction;
    }

    public Vector2d(Distance2d x, Distance2d y) {
        double magnitudeMeters = Math.sqrt(Math.pow(x.toMeters(), 2) + Math.pow(y.toMeters(), 2));
        double directionRadians = Math.atan(y.toMeters()/x.toMeters());
        if (x.toMeters() == 0.0 && y.toMeters() == 0.0) {
            directionRadians = 0.0;
        }
        this.magnitude = Distance2d.fromMeters(magnitudeMeters);
        this.direction = Rotation2d.fromRadians(directionRadians);
    }

    /**
     * Returns the magnitude of the vector (sqrt(x^2+y^2))
     */
    public Distance2d getMagnitude() {
        return this.magnitude;
    }

    /**
     * Returns the direction the vector is facing
     */
    public Rotation2d getDirection() {
        return this.direction;
    }

    /**
     * Basic vector addition between this vector and the given vector
     */
    public Vector2d plus(Vector2d other) {
        double x = magnitude.toMeters() * direction.getCos();
        double y = magnitude.toMeters() * direction.getSin();

        double otherX = other.getMagnitude().toMeters() * other.getDirection().getCos();
        double otherY = other.getMagnitude().toMeters() * other.getDirection().getSin();

        return new Vector2d(
            Distance2d.fromMeters(x + otherX), 
            Distance2d.fromMeters(y + otherY)
        );
    }

    /**
     * Displays the vector's magnitude and direction as a string
     */
    public String toString() {
        return "Magnitude: " + magnitude.toMeters() + ", Direction: " + direction.getDegrees();
    }
}