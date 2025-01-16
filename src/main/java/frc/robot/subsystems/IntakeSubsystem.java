package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;


public class IntakeSubsystem  extends SubsystemBase implements ToggleableSubsystem {
    private SparkMax intakeMotor;
    private SparkMax feederMotor;

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



            intakeMotor = new SparkMax(IntakeConstants.intakeCancoderId, MotorType.kBrushless);
            //intakeMotor.restoreFactoryDefaults();
            intakeMotor.configure(null, null, null);
            intakeMotor.setSmartCurrentLimit(IntakeConstants.INTAKE_CURRENT_LIMIT_A);
            intakeMotor.setInverted(true);
            intakeMotor.setIdleMode(SparkBaseConfig.IdleMode.kCoast);
            
            
            feederMotor = new SparkMax(IntakeConstants.feederCancoderId, MotorType.kBrushless);
            //feederMotor.restoreFactoryDefaults();
            intakeMotor.configure(null, null, null)
            feederMotor.setSmartCurrentLimit(IntakeConstants.FEEDER_CURRENT_LIMIT_A);
            feederMotor.setInverted(true);
            feederMotor.setIdleMode(IdleMode.kBrake);

            m_forwardLimit = feederMotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
            m_reverseLimit = feederMotor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed);

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
        m_forwardLimit.enableLimitSwitch(false);
    }

    public void disableReverseLimitSwitch() {
        m_reverseLimit.enableLimitSwitch(false);
    }

    public void enableLimitSwitch() {
        m_forwardLimit.enableLimitSwitch(true);
    }

    public void enableReverseLimitSwitch() {
        m_reverseLimit.enableLimitSwitch(true);
    }
    
    public boolean hasNote() {
       return (feederMotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen).isPressed()) || 
             (!feederMotor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed).isPressed());
    }

    public boolean noteSettled() {
        return feederMotor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed).isPressed();
    }
}
