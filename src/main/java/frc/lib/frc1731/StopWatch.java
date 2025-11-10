package frc.lib.frc1731;

import edu.wpi.first.util.sendable.*;
import edu.wpi.first.wpilibj.Timer;

public class StopWatch implements Sendable {
    private Timer timer = new Timer();
    private double stopTime = 0d;
    private boolean hasStarted = false;

    /**
     * A timer that counts down from a certain starting point
     */
    public StopWatch(double stopTime) {
        this.stopTime = stopTime;
        this.timer.reset();

        SendableRegistry.addLW(this, "StopWatch");
    }

    /**
     * Starts the timer if it hasn't been started yet
     */
    public void start() {
        if (!hasStarted) {
            this.timer.start();
            hasStarted = true;
        }
    }

    /**
     * Stops and resets the timer
     */
    public void stop() {
        this.timer.stop();
        this.timer.reset();
    }

    /**
     * The amount of time elapsed since started
     */
    public double getElapsed() {
        if (!hasStarted) {
            this.start();
            this.hasStarted = true;
        }
        return this.timer.get();
    }

    /**
     * The amount of time remaining
     */
    public double getRemaining() {
        if (!hasStarted) {
            this.start();
            this.hasStarted = true;
        }
        return this.timer.get() - stopTime;
    }

    /**
     * Whether the indicated elapsed time has passed
     */
    public boolean hasFinished() {
        if (!hasStarted) {
            this.start();
            this.hasStarted = true;
        }
        return this.timer.hasElapsed(stopTime);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("StopWatch");
        builder.addDoubleProperty("Stop Time", 
            () -> stopTime, 
            (time) -> stopTime = time
        );
    }
}