package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.ToggleableSubsystem;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.CANdleConfiguration;


public class LEDSubsystem extends SubsystemBase implements ToggleableSubsystem {

    private boolean enabled;
    private CANdle candle; 
    private CANdleConfiguration candleConfig;

    //set using RGB
    private static final int[] OFF = { 0, 0, 0, 0 };
    private static final int[] RED = { 255, 0, 0, 0 };
    private static final int[] BLUE = { 0, 255, 0, 0 };
    private static final int[] GREEN = { 0, 0, 255, 0 };
    private static final int[] YELLOW = { 0, 255, 255, 0 }; //TODO: (SF) not right RGB values
    private static final int[] WHITE = { 0, 0, 0, 1 }; //need to check this is how we get white LEDs

    @Override
    public boolean isEnabled() {
        return enabled;
    }
    
    public LEDSubsystem(boolean enabled){
        this.enabled = enabled;
        if(!enabled) return;
        initializeLED();
    }

    public void setColor(int red, int green, int blue, int white,int startLED, int numberOfLEDs){
        candle.setLEDs(red, green, blue, white, startLED, numberOfLEDs);
    }

    private void setColor(LEDConstants.LedColor color, int startLED, int numberOfLEDs) {
        int r = 0;
        int g = 0;
        int b = 0;
        int w = 0;
        switch (color) {
          case OFF:   r = OFF[0]; g = OFF[1]; b = OFF[2]; w = OFF[3];break;
          case WHITE:   r = WHITE[0]; g = WHITE[1]; b = WHITE[2]; w = WHITE[3]; break;
          case BLUE:    r = BLUE[0]; g = BLUE[1]; b = BLUE[2]; w = BLUE[3]; break;
          case RED:     r = RED[0]; g = RED[1]; b = RED[2]; w = RED[3];break;
          case GREEN:   r = GREEN[0]; g = GREEN[1]; b = GREEN[2]; w = GREEN[3];break;
          case YELLOW:   r = YELLOW[0]; g = YELLOW[1]; b = YELLOW[2]; w = YELLOW[3];break;
        }
            candle.setLEDs(r, g, b, w, startLED, numberOfLEDs);
        }
    
    private void initializeLED(){
        System.out.println("LEDSubsystem: Starting UP & Initializing LEDs !!!!!!!");

        candle = new CANdle(LEDConstants.CANdleCanId, "rio");
        candleConfig = new CANdleConfiguration();
        candleConfig.stripType = LEDStripType.RGB; //TODO: what type of LED string do we have?
        candleConfig.brightnessScalar = 0.2;
        candleConfig.disableWhenLOS = false;
        candleConfig.statusLedOffWhenActive = true;
        candleConfig.vBatOutputMode = VBatOutputMode.On;
        candleConfig.v5Enabled = false;
        candle.configAllSettings(candleConfig);
    }

    public void setBlink(){
    }
    
    public void intTelop(){
        setColor(LEDConstants.LedColor.GREEN, 0, LEDConstants.maxStringLength);
    }

    public void setLineupTooFarRightScheme(){
        setColor(LEDConstants.LedColor.RED, 0, 1);
        setColor(LEDConstants.LedColor.OFF, 1, 6);
        setColor(LEDConstants.LedColor.RED, 7, 1);
    }
    
    public void setLineupTooFarLeftScheme(){
        setColor(LEDConstants.LedColor.OFF, 0, 3);
        setColor(LEDConstants.LedColor.RED, 3, 2);
        setColor(LEDConstants.LedColor.OFF, 5, 3);
    }

    public void setLineupCenteredScheme(){
        setColor(LEDConstants.LedColor.OFF, 0, 1);
        setColor(LEDConstants.LedColor.GREEN, 1, 2);
        setColor(LEDConstants.LedColor.OFF, 3, 2);
        setColor(LEDConstants.LedColor.GREEN, 5, 2);
        setColor(LEDConstants.LedColor.OFF, 7, 1);
    }

    public void setNoTagFound(){
        setColor(LEDConstants.LedColor.OFF, 0, 8);
    }
    public void turnLineupColorsOff(){
        setColor(LEDConstants.LedColor.OFF, 0, LEDConstants.maxStringLength);
    }
}
