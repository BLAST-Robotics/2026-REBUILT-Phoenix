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

/**
 * Hood. A rotating hood adjusts the launch angle of the shot. An X44 drives it
 * through a 15:24 pulley, 30:18 gear, and a 10T gear meshing a 153T gear
 * segment. A WCP through-bore (CANcoder) senses the hood angle.
 *
 * <p>The hood carries two counter-rollers that pass spheres from the drum to
 * the outside of the robot.
 *
 * <p>Device numbers come from {@link Constants.old}.
 */
public class Hood extends SubsystemBase {
    private final TalonFX m_motor;
    private final CANcoder m_encoder;

    public Hood() {
        m_motor = new TalonFX(Constants.old.Hood.kMotor, Constants.kCANivoreBus);
        m_encoder = new CANcoder(Constants.old.Hood.kEncoder, Constants.kCANivoreBus);

        MotorConfigs.applyPivotConfig(m_motor, m_encoder,
            Constants.Hood.kHoodMotorInverted, Constants.Hood.kHoodSupplyLimit, Constants.Hood.kHoodStatorLimit,
            // sensorToMechanismRatio such that reads are in hood revolutions:
            // encoder (1 turn = full 360deg of absolute) to hood output turns.
            Constants.Hood.kHoodTotalReduction,
            Constants.Hood.kHoodMinDegrees / 360.0,
            Constants.Hood.kHoodMaxDegrees / 360.0);
    }

    /** Command hood angle in degrees via closed-loop position. */
    public void setHoodDegrees(double degrees) {
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
        SmartDashboard.putNumber("Hood/Degrees", getHoodDegrees());
        SmartDashboard.putNumber("Hood/MotorCur", m_motor.getStatorCurrent().getValueAsDouble());
    }
}