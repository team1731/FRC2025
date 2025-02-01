package frc.robot.subsystems.score;

public enum ElevatorPosition {
    HOME(0),
    L1_SCORE(0),
    L2_SCORE(0),
    L3_SCORE(0),
    L4_SCORE(0);

    private double elevatorPosition;
    private ElevatorPosition(double position) {
        elevatorPosition = position;
    }

    public double getPosition() {
        return elevatorPosition;
    }
}
