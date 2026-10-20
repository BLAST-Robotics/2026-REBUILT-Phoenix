package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.hardware.MotorConfigs;

/** Agitator that moves spheres forward toward the shooter. */
public class Hopper extends SubsystemBase {
    private final TalonFX m_agitator;

    private double m_agitatorTargetRps = 0.0;

    public Hopper() {
        m_agitator = new TalonFX(Constants.ID.kAgitator, Constants.kCANBus);
        MotorConfigs.applyVelocityConfig(m_agitator, Constants.Hopper.kAgitatorInverted,
            Constants.Hopper.kAgitatorSupplyLimit, Constants.Hopper.kAgitatorStatorLimit,
            Constants.Hopper.kAgitatorKP, Constants.Hopper.kAgitatorKI, Constants.Hopper.kAgitatorKD,
            0.0, 0.0);

        // Agitator coasts always; only pivots ever brake.
        m_agitator.setNeutralMode(com.ctre.phoenix6.signals.NeutralModeValue.Coast);
    }

/** Sets agitator velocity in mechanism RPS. */
    public void setAgitatorRps(double rps) {
        if (!Constants.Enabled.kHopper) {
            m_agitator.set(0.0);
            return;
        }
        m_agitatorTargetRps = rps;
        double motorRps = MotorConfigs.outputToMotorRps(rps, Constants.Hopper.kAgitatorReduction);
        m_agitator.setControl(new VelocityVoltage(motorRps));
    }

    public void setAgitatorPercentage(double percent) {
        m_agitator.set(Constants.Enabled.kHopper ? percent : 0.0);
    }

    public Command runForward() {
        return run(() -> setAgitatorRps(Constants.Hopper.kAgitatorMaxRPS));
    }

    /**
     * Match feed: idle trickle until the drums are up to speed, then full
     * feed. Reads (not requires) the shooter, so it parallels shootAtRpm.
     * Ends back at idle (same held-request rule as the drums).
     */
    public Command feedWhenReady(frc.robot.subsystems.Shooter shooter,
            double idleRps, double shootRps) {
        return runEnd(() -> setAgitatorRps(shooter.feedReady() ? shootRps : idleRps),
            () -> setAgitatorRps(idleRps));
    }

    /** Reverse the agitator (jams / clearing). */
    public Command runReverse() {
        return run(() -> setAgitatorRps(-Constants.Hopper.kAgitatorMaxRPS * 0.5));
    }

    public Command stop() {
        return runOnce(() -> setAgitatorPercentage(0.0));
    }

    /** Full neutral: zero output + honest target. */
    public void killOutputs() {
        m_agitatorTargetRps = 0.0;
        setAgitatorPercentage(0.0);
    }

    /** Disabled-state hold: neutralize once, then re-assert zero every loop. */
    public Command coastHold() {
        return runOnce(this::killOutputs)
            .andThen(run(() -> setAgitatorPercentage(0.0)));
    }

    @Override
    public void periodic() {
        double agitatorRps = m_agitator.getVelocity().getValueAsDouble()
            / Constants.Hopper.kAgitatorReduction;
        SmartDashboard.putNumber("Hopper/AgitatorRPS", agitatorRps);
        SmartDashboard.putNumber("Hopper/AgitatorTargetRPS", m_agitatorTargetRps);
        SmartDashboard.putBoolean("Hopper/AgitatorAtSpeed",
            m_agitatorTargetRps != 0 && Math.abs(m_agitatorTargetRps - agitatorRps) < 3.0);
        SmartDashboard.putNumber("Hopper/AgitatorCur", m_agitator.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Hopper/AgitatorSupplyCur", m_agitator.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Hopper/AgitatorTempC", m_agitator.getDeviceTemp().getValueAsDouble());
    }
}
