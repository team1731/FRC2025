package frc.lib.frc1678.sim;

import static edu.wpi.first.units.Units.KilogramSquareMeters;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

/**
 * Class for simulating a pivoting system powerd by one or more motors like a rotating arm.
 */
public class PivotSim extends MechanismSim {
	protected final SingleJointedArmSim sim;
	/**
	 * Creates a PivotSim from provided constants.
	 *
	 * @param constants Constants to use for PivotSim.
	 */
	public PivotSim(PivotSimConstants constants) {
		super(constants.gearing);
		sim = new SingleJointedArmSim(
				constants.motor,
				constants.gearing,
				constants.momentOfInertia.in(Units.KilogramSquareMeters),
				constants.armLength.in(Units.Meters),
				constants.mechanismMinHardStop.in(Units.Radians),
				constants.mechanismMaxHardStop.in(Units.Radians),
				constants.simGravity,
				constants.mechanismStartPos.in(Units.Radians));
	}

	@Override
	public void setVoltage(Voltage voltage) {
		sim.setInputVoltage(voltage.in(Units.Volts));
	}

	/**
	 * Constants for creating a PivotSim.
	 */
	public static class PivotSimConstants {
		public DCMotor motor;
		public double gearing;
		public MomentOfInertia momentOfInertia;
		public Distance armLength;
		public Angle mechanismMinHardStop;
		public Angle mechanismMaxHardStop;
		public boolean simGravity;
		public Angle mechanismStartPos;

		public PivotSimConstants withMotor(DCMotor motor) {
			this.motor = motor;
			return this;
		}

		public PivotSimConstants withPhysics(double gearing, double pivotMOI, boolean simGravity) {
			this.gearing = gearing;
			this.momentOfInertia = KilogramSquareMeters.of(pivotMOI);
			this.simGravity = simGravity;
			return this;
		}

		public PivotSimConstants withConstraints(double minDegrees, double maxDegrees, double startDegrees, double armLengthMeters) {
			this.mechanismMinHardStop = Units.Degrees.of(minDegrees);
			this.mechanismMaxHardStop = Units.Degrees.of(maxDegrees);
			this.mechanismStartPos = Units.Degrees.of(startDegrees);
			this.armLength = Units.Meters.of(armLengthMeters);
			return this;
		}
	}

	@Override
	public AngularVelocity getVelocity() {
		return Units.Radians.of(sim.getVelocityRadPerSec()).per(Units.Second);
	}

	@Override
	public Angle getPosition() {
		return Units.Radians.of(sim.getAngleRads());
	}

	@Override
	public Current getStatorCurrent() {
		return Units.Amps.of(sim.getCurrentDrawAmps());
	}

	@Override
	protected void update(Time deltaTime) {
		sim.update(deltaTime.in(Units.Seconds));
	}

	@Override
	public void setState(Angle angle, AngularVelocity velocity) {
		sim.setState(angle.in(Units.Radians), velocity.in(Units.RadiansPerSecond));
	}
}