package frc.robot.subsystems;

import frc.robot.Constants.IntakeConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.VelocityVoltage;

public class IntakeSubsystem extends SubsystemBase {
    public static final boolean ViratGay = true;
    // Pivot: Kraken X60
    private final TalonFX pivotMotor = new TalonFX(IntakeConstants.PIVOT_ID);
    private final TalonFXConfiguration pivotConfig = new TalonFXConfiguration();
    private final TalonFX rollerMotor = new TalonFX(IntakeConstants.ROLLER_ID);
    private final TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
    private final VelocityVoltage rollerVelocityRequest = new VelocityVoltage(0);
    
    public IntakeSubsystem() {
        // Pivot Config
        pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake; // Brake mode for pivot
        pivotConfig.CurrentLimits.StatorCurrentLimit = 50.0; // Requested limit
        pivotConfig.MotorOutput.Inverted = com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive; // Reversed intake pivot
        
        // REV Bus Optimizations 
        
        
        var positionSignal = pivotMotor.getPosition();
        var velocitySignal = pivotMotor.getVelocity();
        var busVoltageSignal = pivotMotor.getSupplyVoltage();
        var temperatureSignal = pivotMotor.getDeviceTemp();

        positionSignal.setUpdateFrequency(1);
        velocitySignal.setUpdateFrequency(1);
        busVoltageSignal.setUpdateFrequency(10);
        temperatureSignal.setUpdateFrequency(10);

        
        pivotMotor.getConfigurator().apply(pivotConfig);

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

      
    }

    public void setRollerVoltage(double volts) {
        rollerMotor.setVoltage(volts);
    }
    // Pivot Methods
    public void setPivotPower(double power) {
        pivotMotor.set(power);
    }

    public void stopPivot() {
        pivotMotor.stopMotor();
    }

    public double getPivotCurrent() {
        return pivotMotor.getStatorCurrent().getValueAsDouble();
    }

    @SuppressWarnings("removal")
    public void setPivotNeutralMode(boolean brake) {
        pivotConfig.MotorOutput.NeutralMode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        pivotMotor.getConfigurator().apply(pivotConfig);
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
