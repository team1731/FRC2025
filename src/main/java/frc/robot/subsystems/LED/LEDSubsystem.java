package frc.robot.subsystems.LED;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.ToggleableSubsystem;
import frc.robot.subsystems.LED.LEDConstants;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.CANdleConfiguration;


public class LEDSubsystem extends SubsystemBase implements ToggleableSubsystem {

    private boolean enabled;
    private CANdle candle; 
    private CANdleConfiguration candleConfig;

    @Override
    public boolean isEnabled() {
        return enabled;
    }
    
    public LEDSubsystem(boolean enabled){
        this.enabled = enabled;
        if(!enabled) return;
        initializeLED();
    }

    public void setColor(int r, int g, int b, int startLED, int numberLED){
        candle.setLEDs(r, g, b, 0, startLED, numberLED);
    }

    public void setBlink(){
    }
    
    private void initializeLED(){
        System.out.println("LEDSubsystem: Starting UP & Initializing LEDs !!!!!!!");

        candle = new CANdle(LEDConstants.CANdleCanId, "rio");
        candleConfig = new CANdleConfiguration();
        candleConfig.stripType = LEDStripType.GRB; //TODO: what type of LED string do we have?
        candleConfig.brightnessScalar = 0.3;
        candleConfig.disableWhenLOS = false;
        candleConfig.statusLedOffWhenActive = true;
        candleConfig.vBatOutputMode = VBatOutputMode.On;
        candleConfig.v5Enabled = false;
        candle.configAllSettings(candleConfig);
    }

    public void intTelop(){
        setColor(0, 255, 0, 0, LEDConstants.maxStringLength);
    }

    public void toFarLeft(){
        setColor(255, 0, 0, 0, 1);
        setColor(0, 0, 0, 1, 6);
        setColor(255, 0, 0, 7, 1);
    }
    
    public void toFarRight(){
        setColor(0, 0, 0, 0, 3);
        setColor(255, 0, 0, 3, 2);
        setColor(0, 0, 0, 5, 3);
    }

    public void centerted(){
        setColor(0, 0, 0, 0, 1);
        setColor(0, 255, 0, 1, 2);
        setColor(0, 0, 0, 3, 2);
        setColor(0, 255, 0, 5, 2);
        setColor(0, 0, 0, 7, 1);
    }
}
