package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.hardware.MotorConfigs;

/**
 * Shooter. Two counter-rotating Kraken x60 drive the drum + flywheels
 * (geared 15:24), and one Talon X44 spindle drives the internal rollers. The
 * drum passes spheres up to the rotating hood rollers.
 *
 * <p>The two x60 motors physically face opposite directions, so one is inverted
 * relative to the other; both spin the drum in the same rotational sense.
 *
 * <p>Device numbers come from {@link Constants.old}.
 */
public class Shooter extends SubsystemBase {
    private final TalonFX m_drumLeft;
    private final TalonFX m_drumRight;
    private final TalonFX m_spindle;

    public Shooter() {
        m_drumLeft = new TalonFX(Constants.old.Shooter.kDrumLeft, Constants.kCANivoreBus);
        m_drumRight = new TalonFX(Constants.old.Shooter.kDrumRight, Constants.kCANivoreBus);
        m_spindle = new TalonFX(Constants.old.Shooter.kSpindle, Constants.kCANivoreBus);

        // Both motors drive the drum; the right-facing one is inverted so they
        // counter-rotate but spin the drum the same way.
        MotorConfigs.applyVelocityConfig(m_drumLeft, false,
            Constants.Shooter.kDrumSupplyLimit, Constants.Shooter.kDrumStatorLimit,
            Constants.Shooter.kDrumKP, Constants.Shooter.kDrumKI, Constants.Shooter.kDrumKD,
            Constants.Shooter.kDrumKV, Constants.Shooter.kDrumKA);
        MotorConfigs.applyVelocityConfig(m_drumRight, true,
            Constants.Shooter.kDrumSupplyLimit, Constants.Shooter.kDrumStatorLimit,
            Constants.Shooter.kDrumKP, Constants.Shooter.kDrumKI, Constants.Shooter.kDrumKD,
            Constants.Shooter.kDrumKV, Constants.Shooter.kDrumKA);

        MotorConfigs.applyVelocityConfig(m_spindle, false,
            Constants.Shooter.kSpindleSupplyLimit, Constants.Shooter.kSpindleStatorLimit,
            Constants.Shooter.kSpindleKP, Constants.Shooter.kSpindleKI, Constants.Shooter.kSpindleKD,
            0.0, 0.0);
    }

    /** Set drum speed in mechanism RPM (both motors driven together). */
    public void setDrumRpm(double rpm) {
        double motorRps = MotorConfigs.outputToMotorRps(rpm / 60.0, Constants.Shooter.kDrumGearReduction);
        m_drumLeft.setControl(new VelocityVoltage(motorRps));
        m_drumRight.setControl(new VelocityVoltage(motorRps));
    }

    /** Set spindle (internal roller) speed in mechanism RPS. */
    public void setSpindleRps(double rps) {
        double motorRps = MotorConfigs.outputToMotorRps(rps, Constants.Shooter.kSpindleReduction);
        m_spindle.setControl(new VelocityVoltage(motorRps));
    }

    /** Percent output on both drum motors. */
    public void setDrumPercentage(double percent) {
        m_drumLeft.set(percent);
        m_drumRight.set(percent);
    }

    /** Percent output on the spindle. */
    public void setSpindlePercentage(double percent) {
        m_spindle.set(percent);
    }

    /** Revol the shooter to the commanded RPM. */
    public Command revToRpm(double rpm) {
        return run(() -> setDrumRpm(rpm));
    }

    /** Run spindle (rollers) during a shot. */
    public Command runSpindle(double rps) {
        return run(() -> setSpindleRps(rps));
    }

    public Command stopAll() {
        return runOnce(() -> {
            setDrumPercentage(0.0);
            setSpindlePercentage(0.0);
        });
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter/DrumRPM",
            m_drumLeft.getVelocity().getValueAsDouble() * 60.0 / Constants.Shooter.kDrumGearReduction);
        SmartDashboard.putNumber("Shooter/DrumCur",
            m_drumLeft.getStatorCurrent().getValueAsDouble()
                + m_drumRight.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/SpindleRPS",
            m_spindle.getVelocity().getValueAsDouble() / Constants.Shooter.kSpindleReduction);
    }
}