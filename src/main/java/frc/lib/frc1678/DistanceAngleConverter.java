package frc.lib.frc1678;

import edu.wpi.first.units.*;
import edu.wpi.first.units.measure.*;

/**
 * Class used store translate distances in the form of angles. Used for elevators to interface with the IO layer which only supports angles.
 */
public class DistanceAngleConverter {
	private final Distance radius;

	public DistanceAngleConverter(Distance radius) {
		this.radius = radius;
	}

	/**
	 * Converts a distance measurement to an equal angle measurement based on radius initialized with.
	 *
	 * @param distance Distance to convert to angle.
	 * @return Angle distance is equivalent to.
	 */
	public Angle toAngle(Distance distance) {
		return Units.Radians.of(distance.in(BaseUnits.DistanceUnit) / radius.baseUnitMagnitude());
	}

	/**
	 * Converts an angle measurement to an equal distance measurement based on radius initialized with.
	 *
	 * @param distance angle to convert to distance.
	 * @return Distance agle is equivalent to.
	 */
	public Distance toDistance(Angle angle) {
		return BaseUnits.DistanceUnit.of(angle.in(Units.Radians) * radius.baseUnitMagnitude());
	}

	/**
	 * Gets an angle unit equivalent to a distance unit with the conversion of the radius initialized with.
	 *
	 * @param unit The distance unit to convert.
	 * @return The distance represented as an AngleUnit
	 */
	public AngleUnit getDistanceUnitAsAngleUnit(DistanceUnit unit) {
		return Units.derive(BaseUnits.AngleUnit)
				.aggregate(toAngle(unit.one()).baseUnitMagnitude())
				.named(unit.name())
				.symbol(unit.symbol())
				.make();
	}

	/**
	 * Gets a distance unit equivalent to a angle unit with the conversion of the radius initialized with.
	 *
	 * @param unit The angle unit to convert.
	 * @return The distance represented as a DistanceUnit
	 */
	public DistanceUnit getAngleUnitAsDistanceUnit(AngleUnit unit) {
		return Units.derive(BaseUnits.DistanceUnit)
				.splitInto(toDistance(unit.one()).baseUnitMagnitude())
				.named(unit.name())
				.symbol(unit.symbol())
				.make();
	}

	public Distance getDrumRadius() {
		return radius;
	}
}