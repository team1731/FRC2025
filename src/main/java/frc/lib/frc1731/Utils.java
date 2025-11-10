package frc.lib.frc1731;

import edu.wpi.first.math.trajectory.Trajectory;

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

    public static void printTrajectory(String name, Trajectory trajectory) {
		System.out.println("\n" + name + ":");
		double duration = trajectory.getTotalTimeSeconds();
		System.out.println("trajectory duration " + duration);
		for (int i = 0; i <= (int) duration * 2; i++) {
			Trajectory.State state = trajectory.sample(i / 2.0);
			System.out
					.println("state " + i + "                 poseMetersX " + state.poseMeters.getTranslation().getX());
			System.out
					.println("state " + i + "                 poseMetersY " + state.poseMeters.getTranslation().getY());
			System.out.println(
					"state " + i + "         poseMetersTheta Deg " + state.poseMeters.getRotation().getDegrees());
			System.out.println("state " + i + "     velocityMetersPerSecond " + state.velocityMetersPerSecond);
		}
		Trajectory.State state = trajectory.sample(duration);
		System.out.println("state (end)             poseMetersX " + state.poseMeters.getTranslation().getX());
		System.out.println("state (end)             poseMetersY " + state.poseMeters.getTranslation().getY());
		System.out.println("state (end)     poseMetersTheta Deg " + state.poseMeters.getRotation().getDegrees());
		System.out.println("state (end) velocityMetersPerSecond " + state.velocityMetersPerSecond);
	}
}