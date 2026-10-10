package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.hardware.MotorConfigs;

/** Rotating hood that adjusts the launch angle. */
public class Hood extends SubsystemBase {
    private final TalonFX m_motor;
    private final CANcoder m_encoder;

    private double m_targetDegrees = 0.0;

    public Hood() {
        m_motor = new TalonFX(Constants.old.Hood.kMotor, Constants.kCANivoreBus);
        m_encoder = new CANcoder(Constants.old.Hood.kEncoder, Constants.kCANivoreBus);

        MotorConfigs.applyPivotConfig(m_motor, m_encoder,
            Constants.Hood.kHoodMotorInverted, Constants.Hood.kHoodSupplyLimit, Constants.Hood.kHoodStatorLimit,
            Constants.Hood.kHoodTotalReduction,
            Constants.Hood.kHoodMinDegrees / 360.0,
            Constants.Hood.kHoodMaxDegrees / 360.0,
            Constants.Hood.kHoodEncoderOffset);
    }

    /** Command hood angle in degrees. */
    public void setHoodDegrees(double degrees) {
        m_targetDegrees = degrees;
        m_motor.setControl(new PositionVoltage(Rotations.of(degrees / 360.0)));
    }

    public void setHoodPercentage(double percent) {
        m_motor.set(percent);
    }

    /** Current hood angle in degrees. */
    public double getHoodDegrees() {
        return m_motor.getPosition().getValueAsDouble() * 360.0;
    }

    public boolean atSetpoint(double degrees, double tolerance) {
        return Math.abs(getHoodDegrees() - degrees) < tolerance;
    }

    public Command setAngle(double degrees) {
        return run(() -> setHoodDegrees(degrees)).until(() -> atSetpoint(degrees, 2.0));
    }

    public Command stop() {
        return runOnce(() -> setHoodPercentage(0.0));
    }

    @Override
    public void periodic() {
        double degrees = getHoodDegrees();
        SmartDashboard.putNumber("Hood/Degrees", degrees);
        SmartDashboard.putNumber("Hood/TargetDeg", m_targetDegrees);
        SmartDashboard.putNumber("Hood/ErrorDeg", m_targetDegrees - degrees);
        SmartDashboard.putBoolean("Hood/AtSetpoint", atSetpoint(m_targetDegrees, 2.0));
        SmartDashboard.putNumber("Hood/MotorCur", m_motor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Hood/SupplyCur", m_motor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Hood/TempC", m_motor.getDeviceTemp().getValueAsDouble());
    }
}