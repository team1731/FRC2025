package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.*;
import frc.lib.frc1678.DistanceAngleConverter;
import frc.lib.frc1678.sim.LinearSim;
import frc.lib.frc1678.sim.LinearSim.LinearSimConstants;
import frc.lib.frc1731.PIDGains;
import frc.lib.frc1731.hardware.motor.PortConfig;
import frc.robot.Constants;

public final class ElevatorConstants {
    // Port Configs
    public final static int masterCanId1 = 21;
    public final static int followerCanId2 = 22;

    public final static InvertedValue elevatorMotor1Direction = InvertedValue.CounterClockwise_Positive;
    public final static InvertedValue elevatorMotor2Direction = InvertedValue.Clockwise_Positive;

    public final static PortConfig leadPortConfig = new PortConfig(
        Constants.CANBUS_NAME,
        ElevatorConstants.masterCanId1,
        ElevatorConstants.elevatorMotor1Direction
    );

    public final static PortConfig followerPortConfig = new PortConfig(
        Constants.CANBUS_NAME,
        ElevatorConstants.followerCanId2,
        ElevatorConstants.elevatorMotor2Direction
    );

    public final static FeedbackConfigs feedbackConfigs = new FeedbackConfigs()
        .withSensorToMechanismRatio(1.0);

    // Physical Elevator Profile
    public final static double gearRatioModifier = (12.0/20);
    public final static double mechGearRatio = 1d;

    public final static Distance drumRadiusInches = Inches.one(); // Inches
    public final static Distance elevatorBaseHeight = Inches.of(36);
    public final static Distance elevatorMaxExtension = Inches.of(60);

    public final static Mass elevatorMass = Pounds.of(30);

    // Simulation Configs
    public final static LinearSim.LinearSimConstants simConstants = 
        new LinearSimConstants()
            .withConverter(new DistanceAngleConverter(Units.Inches.of(2d)))
            .withConstraints(0, elevatorMaxExtension.in(Meters), 0)
            .withMotor(DCMotor.getKrakenX60(2))
            .withPhysics(mechGearRatio, elevatorMass.in(Kilogram), false);

    // Motion Magic Config
    public final static PIDGains motionMagicGains = new PIDGains()
        .setP(4.9)
        .setD(0.0078125)
        .setS(0.02)
        .setV(0.14)
        .setG(0.1);

    public final static double normalElevatorVelocity = 95;
    public final static double normalElevatorAcceleration = 230;

    public final static double slowedElevatorVelocity = 42;
    public final static double slowedElevatorAcceleration = 125;

    // Positions
    public final static Distance elevatorHomePosition = Inches.zero();
    public final static Angle minElevatorPosition = Rotations.zero();
    public final static Angle maxElevatorPosition = Rotations.of(97 * gearRatioModifier);

    public final static double atPositionTolerance = 1.0;
}