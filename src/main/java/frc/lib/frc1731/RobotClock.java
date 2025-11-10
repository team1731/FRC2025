package frc.lib.frc1731;

import edu.wpi.first.wpilibj.Timer;

/**
 * Extension of WPILib's {@code Timer.java} that can handle delta time
 */
public class RobotClock extends Timer {
    private double lastTime = 0.0;
    private double curTime = 0.0;

    /**
     * Change in time between last frame and the current one
     */
    public double dt() {
        return curTime - lastTime;
    }

    /**
     * Periodic method call that updates the current and last times
     */
    public void update() {
        super.start();
        lastTime = curTime;
        curTime = get();
    }
}