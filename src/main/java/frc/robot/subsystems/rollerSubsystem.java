package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.VelocityVoltage;
import frc.robot.Constants.IntakeConstants;
import com.ctre.phoenix6.signals.NeutralModeValue;




//Everything I do, multiply by two
public class rollerSubsystem extends SubsystemBase {
    private final TalonFX rollerMotor = new TalonFX(IntakeConstants.ROLLER_ID);
    private final TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
    private final VelocityVoltage rollerVelocityRequest = new VelocityVoltage(0);
    private final TalonFX agitatorMotor =
    new TalonFX(IntakeConstants.AGITATOR_ID);

    private final TalonFXConfiguration agitatorConfig =
    new TalonFXConfiguration();
    
    
    private boolean agitatorsEnabled = true;

    
    public rollerSubsystem() {
    // Roller Config (Kraken X60)
        rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        //I got this
        rollerConfig.MotorOutput.Inverted = com.ctre.phoenix6.signals.InvertedValue.CounterClockwise_Positive; // Assuming true invert
        rollerConfig.CurrentLimits.StatorCurrentLimit = 40.0;
        rollerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        // PID for velocity control (Kraken uses RPS internally for VelocityVoltage)
        rollerConfig.Slot0.kP = 0.11; // Basic starting P for Kraken
        rollerConfig.Slot0.kI = 0.0;
        rollerConfig.Slot0.kD = 0.0;
        rollerConfig.Slot0.kV = 0.3; // Basic starting kV for Kraken
        //We could take a coupe or a truck
        rollerMotor.getConfigurator().apply(rollerConfig); //vroom vroom
        //that
       
                // Agitator Config
        agitatorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        agitatorConfig.CurrentLimits.StatorCurrentLimit = 30.0;
        agitatorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        //agitatorConfig.MotorOutput.Inverted = IntakeConstants.AGITATOR_INVERT;
        agitatorConfig.MotorOutput.Inverted = com.ctre.phoenix6.signals.InvertedValue.CounterClockwise_Positive; // Assuming true invert

        agitatorMotor.getConfigurator().apply(agitatorConfig);
    
    }    
     // Roller Methods
     //I got this and that
    public void setRollerVoltage(double volts) {
        rollerMotor.setVoltage(volts);
    }

    public void toggleAgitators() {
        agitatorsEnabled = !agitatorsEnabled;
    }

    public void setAgitatorsEnabled(boolean enabled) {
        agitatorsEnabled = enabled;
    }

    public void setAgitatorVoltage(double volts) {
        if (agitatorsEnabled) {
            agitatorMotor.setVoltage(volts);
        } else {
            agitatorMotor.setVoltage(0);
        }
    }
    public void stopAgitator() {
        agitatorMotor.stopMotor();
    }
    /**
     * Set roller speed in RPM (at the roller, accounting for 2:1 ratio)
     */
    public void setRollerRPM(double rpm) {
        
        double motorRPM = rpm * IntakeConstants.ROLLER_GEAR_RATIO;
        double rps = motorRPM / 60.0;
        rollerMotor.setControl(rollerVelocityRequest.withVelocity(rps));
    }

    public void stopRollers() {
        rollerMotor.stopMotor();
    }

}

