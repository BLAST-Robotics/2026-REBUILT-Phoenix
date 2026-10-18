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
 */
public class Shooter extends SubsystemBase {
    private final TalonFX m_drumLeft;
    private final TalonFX m_drumRight;
    private final TalonFX m_spindle;

    private double m_drumTargetRpm = 0.0;
    private double m_drumCmdRpm = 0.0;
    private double m_spindleTargetRps = 0.0;

    public Shooter() {
        m_drumLeft = new TalonFX(Constants.ID.kDrumLeft, Constants.kCANBus);
        m_drumRight = new TalonFX(Constants.ID.kDrumRight, Constants.kCANBus);
        m_spindle = new TalonFX(Constants.ID.kSpindle, Constants.kCANBus);

        // Right motor is inverted so both spin the drum the same way.
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

        // Spin-down motors coast always; only pivots ever brake.
        m_drumLeft.setNeutralMode(com.ctre.phoenix6.signals.NeutralModeValue.Coast);
        m_drumRight.setNeutralMode(com.ctre.phoenix6.signals.NeutralModeValue.Coast);
        m_spindle.setNeutralMode(com.ctre.phoenix6.signals.NeutralModeValue.Coast);
    }

    /** Set drum speed in mechanism RPM, slewed (0.5 s full-range ramp). */
    public void setDrumRpm(double rpm) {
        m_drumTargetRpm = rpm;
        // Slew the live command so the drums spool over ~0.5 s instead of
        // slamming to target (brownout + belt shock). 20 ms loop assumed.
        double step = Constants.Shooter.kDrumRampRpmPerSec * 0.02;
        double err = rpm - m_drumCmdRpm;
        m_drumCmdRpm += Math.max(-step, Math.min(step, err));
        double motorRps = MotorConfigs.outputToMotorRps(m_drumCmdRpm / 60.0,
            Constants.Shooter.kDrumGearReduction);
        m_drumLeft.setControl(new VelocityVoltage(motorRps));
        m_drumRight.setControl(new VelocityVoltage(motorRps));
    }

    /** Set spindle (internal roller) speed in mechanism RPS. */
    public void setSpindleRps(double rps) {
        m_spindleTargetRps = rps;
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

    /** Rev the shooter to the commanded RPM. Ends stopped (see drum note). */
    public Command revToRpm(double rpm) {
        return runEnd(() -> setDrumRpm(rpm), this::hardZeroDrums);
    }

    /** Run spindle (rollers) during a shot. Ends stopped (see drum note). */
    public Command runSpindle(double rps) {
        return runEnd(() -> setSpindleRps(rps), () -> setSpindleRps(0.0));
    }

    /** Hard drum stop: bypasses the spool ramp, zeroes command + control. */
    private void hardZeroDrums() {
        m_drumTargetRpm = 0.0;
        m_drumCmdRpm = 0.0;
        m_drumLeft.setControl(new VelocityVoltage(0.0));
        m_drumRight.setControl(new VelocityVoltage(0.0));
    }

    /** Rev drums AND run spindle together in one command (single requirement). */
    public Command revAndFeed(double rpm, double rps) {
        return runEnd(() -> {
            setDrumRpm(rpm);
            setSpindleRps(rps);
        }, () -> {
            hardZeroDrums();
            setSpindleRps(0.0);
        });
    }

    /** True when the drums are within 250 RPM of a nonzero target. */
    public boolean drumAtSpeed() {
        if (m_drumTargetRpm == 0.0) {
            return false;
        }
        double leftRpm = m_drumLeft.getVelocity().getValueAsDouble() * 60.0
            / Constants.Shooter.kDrumGearReduction;
        double rightRpm = m_drumRight.getVelocity().getValueAsDouble() * 60.0
            / Constants.Shooter.kDrumGearReduction;
        return Math.abs(m_drumTargetRpm - (leftRpm + rightRpm) / 2.0) < 250.0;
    }

    private boolean m_feedLatched = false;

    /** Clear the feed latch (call on shoot-press). */
    public void resetFeedLatch() {
        m_feedLatched = false;
    }

    /**
     * Latched readiness: opens on drumAtSpeed, then STAYS open for the rest
     * of the trigger pull so ball-load sag can't chatter the feed on/off.
     */
    public boolean feedReady() {
        if (drumAtSpeed()) {
            m_feedLatched = true;
        }
        return m_feedLatched;
    }

    /**
     * Match shooting: drums always rev to target; spindle idles until the
     * drums are up to speed AND the hood is on angle, then full feed.
     * One requirement. Ends: drums to zero, spindle back to idle (ending a
     * closed-loop command does NOT stop the motor otherwise — the Talon
     * holds its last request and "shooting never stops").
     */
    public Command shootAtRpm(double rpm, double spindleShootRps, double spindleIdleRps,
            frc.robot.subsystems.Hood hood) {
        return runEnd(() -> {
            setDrumRpm(rpm);
            boolean ready = feedReady() && hood.hoodAtTarget();
            edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putBoolean(
                "Shooter/FeedGate", ready);
            setSpindleRps(ready ? spindleShootRps : spindleIdleRps);
        }, () -> {
            hardZeroDrums();
            setSpindleRps(spindleIdleRps);
        });
    }

    public Command stopAll() {
        return runOnce(() -> {
            hardZeroDrums();
            m_spindleTargetRps = 0.0;
            setSpindlePercentage(0.0);
        });
    }

    /** Full neutral: zero outputs (not zero-velocity hold) + honest targets. */
    public void killOutputs() {
        m_drumTargetRpm = 0.0;
        m_spindleTargetRps = 0.0;
        setDrumPercentage(0.0);
        setSpindlePercentage(0.0);
        resetFeedLatch();
    }

    /** Disabled-state hold: neutralize once, then re-assert zero every loop. */
    public Command coastHold() {
        return runOnce(this::killOutputs)
            .andThen(run(() -> {
                setDrumPercentage(0.0);
                setSpindlePercentage(0.0);
            }));
    }

    @Override
    public void periodic() {
        double drumLeftRpm = m_drumLeft.getVelocity().getValueAsDouble() * 60.0
            / Constants.Shooter.kDrumGearReduction;
        double drumRightRpm = m_drumRight.getVelocity().getValueAsDouble() * 60.0
            / Constants.Shooter.kDrumGearReduction;
        double drumAvgRpm = (drumLeftRpm + drumRightRpm) / 2.0;
        double spindleRps = m_spindle.getVelocity().getValueAsDouble()
            / Constants.Shooter.kSpindleReduction;

        SmartDashboard.putNumber("Shooter/DrumRPM", drumAvgRpm);
        SmartDashboard.putNumber("Shooter/DrumTargetRPM", m_drumTargetRpm);
        SmartDashboard.putNumber("Shooter/DrumErrorRPM", m_drumTargetRpm - drumAvgRpm);
        SmartDashboard.putBoolean("Shooter/DrumAtSpeed",
            m_drumTargetRpm != 0.0 && Math.abs(m_drumTargetRpm - drumAvgRpm) < 250.0);
        SmartDashboard.putNumber("Shooter/DrumLeftRPM", drumLeftRpm);
        SmartDashboard.putNumber("Shooter/DrumRightRPM", drumRightRpm);
        SmartDashboard.putNumber("Shooter/DrumCur",
            m_drumLeft.getStatorCurrent().getValueAsDouble()
                + m_drumRight.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/DrumSupplyCur",
            m_drumLeft.getSupplyCurrent().getValueAsDouble()
                + m_drumRight.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/DrumTempC",
            m_drumLeft.getDeviceTemp().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/SpindleRPS", spindleRps);
        SmartDashboard.putNumber("Shooter/SpindleTargetRPS", m_spindleTargetRps);
        SmartDashboard.putNumber("Shooter/SpindleErrorRPS", m_spindleTargetRps - spindleRps);
        SmartDashboard.putBoolean("Shooter/SpindleAtSpeed",
            m_spindleTargetRps != 0.0 && Math.abs(m_spindleTargetRps - spindleRps) < 5.0);
        SmartDashboard.putNumber("Shooter/SpindleCur",
            m_spindle.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/SpindleTempC",
            m_spindle.getDeviceTemp().getValueAsDouble());
    }
}