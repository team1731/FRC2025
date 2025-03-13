package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.ToggleableSubsystem;
import frc.robot.subsystems.climb.ClimbConstants;

public class ClimbSensorSubsystem extends SubsystemBase implements ToggleableSubsystem{
    
    private DigitalInput sensor1;
    private DigitalInput sensor2;

    private boolean enabled;

    @Override
    public boolean isEnabled() {
        return enabled;
    }

    public ClimbSensorSubsystem(boolean enabled) {
        this.enabled = enabled;
        if (!enabled) return;
        initializeSensors();
    }

    public boolean isLatched() {
        if(isSensor1Triggered() && isSensor2Triggered()){ 
            return true;
        } else {
            return false;
        }
    }

    private void initializeSensors(){
       // System.out.println("ClimbSensorSubsystem: Starting UP & Initializing Sensors !!!!!!");
        sensor1 = new DigitalInput(ClimbConstants.climbSensor1Dio);
        sensor2 = new DigitalInput(ClimbConstants.climbSensor2Dio);
    }

    private boolean isSensor1Triggered() {
        return !sensor1.get();  // inverted because of how the sensors report their state
    }

    private boolean isSensor2Triggered() {
        return !sensor2.get();  // inverted because of how the sensors report their state
    }

    public void periodic(){
        if (!enabled) return;

        log();
    }

    private void log(){
        SmartDashboard.putBoolean("ClimbSensor1", sensor1.get());
        SmartDashboard.putBoolean("ClimbSensor2", sensor2.get());
        SmartDashboard.putBoolean("Climb Latched", isLatched());
    }
}
