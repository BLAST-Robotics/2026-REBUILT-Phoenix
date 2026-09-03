package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.hardware.MotorConfigs;

/**
 * Hopper / agitator. A single motor drives a paddle agitator
 * (16T -> 24T pulley) that moves spheres forward (counter-clockwise positive)
 * toward the shooter.
 *
 * <p>Device number comes from {@link Constants.old}.
 */
public class Hopper extends SubsystemBase {
    private final TalonFX m_agitator;

    public Hopper() {
        m_agitator = new TalonFX(Constants.old.Hopper.kAgitator, Constants.kCANivoreBus);
        MotorConfigs.applyVelocityConfig(m_agitator, Constants.Hopper.kAgitatorInverted,
            Constants.Hopper.kAgitatorSupplyLimit, Constants.Hopper.kAgitatorStatorLimit,
            Constants.Hopper.kAgitatorKP, Constants.Hopper.kAgitatorKI, Constants.Hopper.kAgitatorKD,
            0.0, 0.0);
    }

    /** Sets agitator velocity in mechanism RPS (positive = forward). */
    public void setAgitatorRps(double rps) {
        double motorRps = MotorConfigs.outputToMotorRps(rps, Constants.Hopper.kAgitatorReduction);
        m_agitator.setControl(new VelocityVoltage(motorRps));
    }

    public void setAgitatorPercentage(double percent) {
        m_agitator.set(percent);
    }

    /** Runs the agitator forward to feed spheres. */
    public Command runForward() {
        return run(() -> setAgitatorRps(Constants.Hopper.kAgitatorMaxRPS));
    }

    /** Reverses the agitator (jams / clearing). */
    public Command runReverse() {
        return run(() -> setAgitatorRps(-Constants.Hopper.kAgitatorMaxRPS * 0.5));
    }

    public Command stop() {
        return runOnce(() -> setAgitatorPercentage(0.0));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Hopper/AgitatorRPS",
            m_agitator.getVelocity().getValueAsDouble() / Constants.Hopper.kAgitatorReduction);
        SmartDashboard.putNumber("Hopper/AgitatorCur", m_agitator.getStatorCurrent().getValueAsDouble());
    }
}
