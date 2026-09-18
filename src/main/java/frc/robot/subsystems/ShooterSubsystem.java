package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
    private final TalonFX rightMotor = new TalonFX(ShooterConstants.RIGHT_MOTOR_ID);
    private final TalonFX leftMotor = new TalonFX(ShooterConstants.LEFT_MOTOR_ID);
    private final TalonFX indexer = new TalonFX(ShooterConstants.INDEXER_MOTOR_ID);
    private final TalonFXConfiguration indexerConfig = new TalonFXConfiguration();

    private final VelocityVoltage velocityRequest = new VelocityVoltage(0);
    private final VoltageOut voltageRequest = new VoltageOut(0);
    
    private boolean indexerStarted = false;

    @SuppressWarnings("removal")
    public ShooterSubsystem() {
        TalonFXConfiguration rightConfig = new TalonFXConfiguration();
        rightConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        rightConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        
        // Add current limits
        rightConfig.CurrentLimits.StatorCurrentLimit = 70.0;
        rightConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        rightConfig.CurrentLimits.SupplyCurrentLimit = 70.0;
        rightConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        rightConfig.Slot0.kV = ShooterConstants.SHOOTER_kV;
        rightConfig.Slot0.kP = ShooterConstants.SHOOTER_kP;
        
        rightConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 1.0;
        rightConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 1.0;

        rightMotor.getConfigurator().apply(rightConfig);

        // Left motor follows right motor but is inverted relative to it
        leftMotor.setControl(new Follower(ShooterConstants.RIGHT_MOTOR_ID, MotorAlignmentValue.Opposed));
        
        // Ensure left motor starts in Coast and applies current limits
        TalonFXConfiguration leftConfig = new TalonFXConfiguration();
        leftConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        leftConfig.CurrentLimits.StatorCurrentLimit = 70.0;
        leftConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        leftConfig.CurrentLimits.SupplyCurrentLimit = 70.0;
        leftConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        leftMotor.getConfigurator().apply(leftConfig);
        
        // Config indexer (Clockwise positive, Coast mode)
        indexerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // Clockwise positive
        indexerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        indexerConfig.CurrentLimits.StatorCurrentLimit = 40.0;
        indexerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        indexerConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 1.0;
        indexerConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 1.0;
        
        // REV Bus Optimizations for Indexer
      

        indexerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        //I got this
        indexerConfig.MotorOutput.Inverted = com.ctre.phoenix6.signals.InvertedValue.CounterClockwise_Positive; // Assuming true invert
        indexerConfig.CurrentLimits.StatorCurrentLimit = 40.0;
        indexerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        var positionSignal = indexer.getPosition();
        var velocitySignal = indexer.getVelocity();
        var busVoltageSignal = indexer.getSupplyVoltage();
        var temperatureSignal = indexer.getDeviceTemp();

        positionSignal.setUpdateFrequency(1);
        velocitySignal.setUpdateFrequency(1);
        busVoltageSignal.setUpdateFrequency(10);
        temperatureSignal.setUpdateFrequency(10);

        indexer.getConfigurator().apply(indexerConfig); //vroom vroom


        // CTRE Bus Optimizations
        // High frequency for control/telemetry on leader
        rightMotor.getVelocity().setUpdateFrequency(100);
        rightMotor.getMotorVoltage().setUpdateFrequency(50);
        
        // Low frequency for non-essential status on leader
        rightMotor.getDeviceTemp().setUpdateFrequency(4);
        rightMotor.getSupplyVoltage().setUpdateFrequency(4);
        rightMotor.getFault_Hardware().setUpdateFrequency(4);
        rightMotor.getAcceleration().setUpdateFrequency(4);
        rightMotor.getDutyCycle().setUpdateFrequency(4);
        rightMotor.getTorqueCurrent().setUpdateFrequency(4);

        // Follower needs very little telemetry back to RIO
        leftMotor.getVelocity().setUpdateFrequency(4);
        leftMotor.getMotorVoltage().setUpdateFrequency(4);
        leftMotor.getDeviceTemp().setUpdateFrequency(4);
        leftMotor.getSupplyVoltage().setUpdateFrequency(4);
        leftMotor.getFault_Hardware().setUpdateFrequency(4);
    }

    /**
     * Set the shooter velocity in rotations per second.
     * @param rotationsPerSecond Target RPS
     */
    public void setSpeed(double rotationsPerSecond) {
        rightMotor.setControl(velocityRequest.withVelocity(rotationsPerSecond));
        
        //if (!indexerStarted && rightMotor.getVelocity().getValueAsDouble() >= 0.95 * rotationsPerSecond) {
        //    indexerStarted = true;
        //}
        
        //if (indexerStarted) {
        //    indexer.set(-ShooterConstants.INDEXER_POWER);
        //} else {
            indexer.set(0);
        //}
    }

    /**
     * Set the shooter motor voltage directly.
     * @param volts Target voltage
     */
    public void setVoltage(double volts) {
        rightMotor.setControl(voltageRequest.withOutput(volts));
        indexer.set(-ShooterConstants.INDEXER_POWER);
    }

    /**
     * Stop the shooter motors.
     */
    public void stop() {
        rightMotor.stopMotor();
        indexer.stopMotor();
        indexerStarted = false;
    }

    /**
     * Toggle the neutral mode of the motors.
     * @param brake True for Brake mode, false for Coast mode.
     */
    public void setNeutralMode(boolean brake) {
        NeutralModeValue mode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        
        TalonFXConfiguration config = new TalonFXConfiguration();
        
        // We only really need to apply the neutral mode to both as the follower might not inherit this specific setting in all P6 versions
        rightMotor.getConfigurator().refresh(config.MotorOutput);
        config.MotorOutput.NeutralMode = mode;
        rightMotor.getConfigurator().apply(config.MotorOutput);

        leftMotor.getConfigurator().refresh(config.MotorOutput);
        config.MotorOutput.NeutralMode = mode;
        leftMotor.getConfigurator().apply(config.MotorOutput);
    }

    public double getActualVelocity() {
        return rightMotor.getVelocity().getValueAsDouble();
    }

    public double getAppliedVoltage() {
        return rightMotor.getMotorVoltage().getValueAsDouble();
    }

    public boolean isUpToSpeed(double target) {
        return Math.abs(getActualVelocity() - target) < ShooterConstants.VELOCITY_THRESHOLD;
    }

    @Override
    public void periodic() {
        // Periodic telemetry could go here
    }
}
