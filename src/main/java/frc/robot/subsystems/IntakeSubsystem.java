package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.LimitSwitchConfigAccessor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;


public class IntakeSubsystem  extends SubsystemBase implements ToggleableSubsystem {
    private SparkMax intakeMotor;
    private SparkMax feederMotor;

    private SparkMaxConfig feederConfig = new SparkMaxConfig(); //Create a config for the feeder motor

    private SparkLimitSwitch m_forwardLimit;
    private SparkLimitSwitch m_reverseLimit;

    private boolean enabled;
    
    @Override
    public boolean isEnabled() {
        return enabled;
    }

    public IntakeSubsystem(boolean enabled) {
        this.enabled = enabled;
        initializeIntakeMotor();
    }

    private void initializeIntakeMotor() {
        if (enabled) {
            System.out.println("IntakeSubsystem: Starting up & Initializine Intake motors !!!!!!!!!!!!!!");

            SparkMaxConfig intakeConfig = new SparkMaxConfig();     //Create a config for the intake motor
            intakeConfig.smartCurrentLimit(IntakeConstants.INTAKE_CURRENT_LIMIT_A);     //pass the intake current limit from the generated values in Constants
            intakeConfig.inverted(enabled);     //inverts the intake motor
            intakeConfig.idleMode(IdleMode.kBrake);     //prevents the motor from coasting 

            intakeMotor = new SparkMax(IntakeConstants.intakeCancoderId, MotorType.kBrushless);     
            intakeMotor.configure(null, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);      //resets all configuration
            intakeMotor.configure(intakeConfig, null, null);        //applies the configurations
            //restoreFactoryDefaults();
            //intakeMotor.setSmartCurrentLimit(IntakeConstants.INTAKE_CURRENT_LIMIT_A);
            //intakeMotor.setInverted(true);
            //intakeMotor.setIdleMode(IdleMode.kBrake);
            
            
            feederConfig.smartCurrentLimit(IntakeConstants.FEEDER_CURRENT_LIMIT_A);     //pass the intake current limit from the generated values in Constants
            feederConfig.inverted(enabled);     //inverts the feeder motor
            feederConfig.idleMode(IdleMode.kBrake);     //prevents the motor from coasting
            feederConfig.limitSwitch.forwardLimitSwitchType(LimitSwitchConfig.Type.kNormallyOpen);
            feederConfig.limitSwitch.reverseLimitSwitchType(LimitSwitchConfig.Type.kNormallyClosed);

            feederMotor = new SparkMax(IntakeConstants.intakeCancoderId, MotorType.kBrushless);     
            feederMotor.configure(null, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);      //resets all configuration
            feederMotor.configure(feederConfig, null, null);        //applies the configurations
            //restoreFactoryDefaults();
            //feederMotor.setSmartCurrentLimit(IntakeConstants.FEEDER_CURRENT_LIMIT_A);
            //feederMotor.setInverted(true);
            //feederMotor.setIdleMode(IdleMode.kBrake);

            m_forwardLimit = feederMotor.getForwardLimitSwitch();
            m_reverseLimit = feederMotor.getReverseLimitSwitch();      


            // LimitSwitchConfig forwardLimitSwitchToggleOn = new LimitSwitchConfig();
            // forwardLimitSwitchToggleOn.forwardLimitSwitchEnabled(true);
            // LimitSwitchConfig forwardLimitSwitchToggleOff = new LimitSwitchConfig();
            // forwardLimitSwitchToggleOff.forwardLimitSwitchEnabled(false);

            // LimitSwitchConfig reverseLimitSwitchToggleOn = new LimitSwitchConfig();
            // reverseLimitSwitchToggleOn.reverseLimitSwitchEnabled(true);
            // LimitSwitchConfig reverseLimitSwitchToggleOff = new LimitSwitchConfig();
            // reverseLimitSwitchToggleOff.reverseLimitSwitchEnabled(false);

            enableReverseLimitSwitch();
            enableLimitSwitch();
        }
    }

    public boolean forwardLimitReached(){
        return m_forwardLimit.isPressed();
    }

  public void intakeState(double intakeSpeed) {
        if (enabled) {
            intakeMotor.set(intakeSpeed);
        }
    }

    public void feedState(double feedSpeed) {
        if (enabled) {
            feederMotor.set(feedSpeed);
        }
    }
   
    public void periodic() {
    }

    public void disableLimitSwitch() {
       // m_forwardLimit.enableLimitSwitch(false);
       feederConfig.limitSwitch.forwardLimitSwitchEnabled(false);
       feederMotor.configure(feederConfig, null, null); 
    }

    public void disableReverseLimitSwitch() {
       // m_reverseLimit.enableLimitSwitch(false);
       feederConfig.limitSwitch.reverseLimitSwitchEnabled(false);
       feederMotor.configure(feederConfig, null, null); 
    }

    public void enableLimitSwitch() {
      //  m_forwardLimit.enableLimitSwitch(true);
      feederConfig.limitSwitch.forwardLimitSwitchEnabled(true);
      feederMotor.configure(feederConfig, null, null);

    }

    public void enableReverseLimitSwitch() {
       // m_reverseLimit.enableLimitSwitch(true);
       feederConfig.limitSwitch.reverseLimitSwitchEnabled(true);
       feederMotor.configure(feederConfig, null, null); 
    }
    
    public boolean hasNote() {
       return (m_forwardLimit.isPressed()) || (!m_reverseLimit.isPressed());
    }

    public boolean noteSettled() {
        return m_reverseLimit.isPressed();
    }
}
