package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

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
    // Rising-edge timer for the overramp: when the drums were stopped, this
    // records the moment they started revving (setDrumRpm).
    private double m_overrampStart = 0.0;
    private double m_spindleTargetRps = 0.0;

    public Shooter() {
        m_drumLeft = new TalonFX(Constants.ID.kDrumLeft, Constants.kCANBus);
        m_drumRight = new TalonFX(Constants.ID.kDrumRight, Constants.kCANBus);
        m_spindle = new TalonFX(Constants.ID.kSpindle, Constants.kCANBus);

        // Left is the closed-loop leader. Right is a FOLLOWER on the same shaft,
        // mounted facing the opposite direction, so it mirrors the leader with
        // Opposed alignment (its own output inversion stays default).
        MotorConfigs.applyVelocityConfig(m_drumLeft, false,
            Constants.Shooter.kDrumSupplyLimit, Constants.Shooter.kDrumStatorLimit,
            Constants.Shooter.kDrumKP, Constants.Shooter.kDrumKI, Constants.Shooter.kDrumKD,
            Constants.Shooter.kDrumKS, Constants.Shooter.kDrumKV, Constants.Shooter.kDrumKA,
            Constants.Shooter.kDrumVelocityFilterS);
        MotorConfigs.applyVelocityConfig(m_drumRight, false,
            Constants.Shooter.kDrumSupplyLimit, Constants.Shooter.kDrumStatorLimit,
            Constants.Shooter.kDrumKP, Constants.Shooter.kDrumKI, Constants.Shooter.kDrumKD,
            Constants.Shooter.kDrumKS, Constants.Shooter.kDrumKV, Constants.Shooter.kDrumKA,
            Constants.Shooter.kDrumVelocityFilterS);

        MotorConfigs.applyVelocityConfig(m_spindle, false,
            Constants.Shooter.kSpindleSupplyLimit, Constants.Shooter.kSpindleStatorLimit,
            Constants.Shooter.kSpindleKP, Constants.Shooter.kSpindleKI, Constants.Shooter.kSpindleKD,
            0.0, 0.0, 0.0, 0.0);

        // Spool ramp in HARDWARE (replaces the old code-side slew): the
        // MotionMagic profile ramps the velocity setpoint at kDrumMMAccelRps2
        // with the jerk limit, giving a smooth trapezoidal spool.
        m_drumLeft.getConfigurator().apply(new com.ctre.phoenix6.configs.MotionMagicConfigs()
            .withMotionMagicAcceleration(Constants.Shooter.kDrumMMAccelRps2)
            .withMotionMagicJerk(Constants.Shooter.kDrumMMJerkRps3));

        // Spin-down motors coast always; only pivots ever brake.
        m_drumLeft.setNeutralMode(com.ctre.phoenix6.signals.NeutralModeValue.Coast);
        m_drumRight.setNeutralMode(com.ctre.phoenix6.signals.NeutralModeValue.Coast);
        m_spindle.setNeutralMode(com.ctre.phoenix6.signals.NeutralModeValue.Coast);

        // Right drum mirrors the leader on the shared shaft. The motors are
        // mounted facing opposite directions, so Opposed negates the leader's
        // output — exactly what the old "right motor inverted + same setpoint"
        // paired loop did, but with a single loop so they cannot fight.
        m_drumRight.setControl(
            new Follower(m_drumLeft.getDeviceID(), MotorAlignmentValue.Opposed));
        // A follower mirrors the leader through a status signal, and CTRE
        // requires that signal to update at >= 20 Hz. Default is slower, so
        // pin the output signal fast (voltage, since the drum uses
        // VelocityVoltage) or the right drum will lag/judder.
        m_drumLeft.getMotorVoltage().setUpdateFrequency(100.0);
        m_drumLeft.getDutyCycle().setUpdateFrequency(100.0);
        // Telemetry + drumAtSpeed average both drums, so both velocity
        // signals must be fresh (the follower's is not a control signal).
        m_drumLeft.getVelocity().setUpdateFrequency(100.0);
        m_drumRight.getVelocity().setUpdateFrequency(100.0);
    }

    /**
     * Set drum speed as a POSITIVE shooter RPM magnitude. The drum only ever
     * runs the feeding way, so the hardware direction is applied here (see
     * kDrumDriveSign) and no caller deals with sign. The ramp is the
     * hardware MotionMagic profile (kDrumMMAccelRps2). Targets are clamped to
     * kDrumMaxRPM.
     *
     * <p>OVERRAMP: when this rev starts from a stopped drum, the commanded
     * RPM is raised above target and decays back over kDrumOverrampTimeS, so
     * a fed ball can't sag through the at-speed window. The at-speed gate
     * always compares against the TRUE target, never the overramped one.
     * Targets beyond the ceiling are clamped (kDrumMaxRPM).
     */
    public void setDrumRpm(double rpm) {
        if (!Constants.Enabled.kShooter) {
            m_drumLeft.set(0.0);
            return;
        }
        double target = Math.min(Constants.Shooter.kDrumMaxRPM,
            Math.max(0.0, Math.abs(rpm) + Constants.Shooter.kDrumRpmOffset));
        if (target == 0.0) {
            hardZeroDrums();
            return;
        }
        double now = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
        if (m_drumTargetRpm <= 0.0) {
            m_overrampStart = now;
        }
        double overramp = 0.0;
        double dt = now - m_overrampStart;
        if (dt < Constants.Shooter.kDrumOverrampTimeS && m_drumTargetRpm > 0.0) {
            overramp = Constants.Shooter.kDrumOverrampRpm
                * (1.0 - dt / Constants.Shooter.kDrumOverrampTimeS);
        }
        m_drumTargetRpm = target;
        // Leader only; the right drum follows it on the shared shaft.
        // MotionMagic profiles the ramp; the velocity loop + Ks hold it.
        double motorRps = Constants.Shooter.kDrumDriveSign
            * MotorConfigs.outputToMotorRps((target + overramp) / 60.0,
                Constants.Shooter.kDrumGearReduction);
        m_drumLeft.setControl(
            new MotionMagicVelocityVoltage(motorRps).withEnableFOC(true));
    }

    /** Set spindle (internal roller) speed in mechanism RPS. */
    public void setSpindleRps(double rps) {
        if (!Constants.Enabled.kShooter) {
            m_spindle.set(0.0);
            return;
        }
        m_spindleTargetRps = rps;
        double motorRps = MotorConfigs.outputToMotorRps(rps, Constants.Shooter.kSpindleReduction);
        m_spindle.setControl(new com.ctre.phoenix6.controls.VelocityVoltage(motorRps));
    }

    /** Percent output on the drum (leader; the right drum follows). */
    public void setDrumPercentage(double percent) {
        m_drumLeft.set(Constants.Enabled.kShooter ? percent : 0.0);
    }

    /** Percent output on the spindle. */
    public void setSpindlePercentage(double percent) {
        m_spindle.set(Constants.Enabled.kShooter ? percent : 0.0);
    }

    /** Rev the shooter to the commanded RPM. Ends stopped (see drum note). */
    public Command revToRpm(double rpm) {
        return runEnd(() -> setDrumRpm(rpm), this::hardZeroDrums);
    }

    /** Run spindle (rollers) during a shot. Ends stopped (see drum note). */
    public Command runSpindle(double rps) {
        return runEnd(() -> setSpindleRps(rps), this::hardZeroSpindle);
    }

    /**
     * Hard drum stop: bypasses the spool ramp, zeroes command + control.
     * Open-loop zero (duty 0), NOT a 0-RPM VelocityVoltage hold: the velocity
     * loop dithers the drums back-and-forth at standstill (friction + encoder
     * noise) — that was the post-shot buzz. Coast neutral mode takes over.
     */
    private void hardZeroDrums() {
        m_drumTargetRpm = 0.0;
        // Leader only; the follower mirrors the zero.
        m_drumLeft.set(0.0);
    }

    /**
     * Hard spindle stop (same 0-RPM-hold dither fix as the drums): open-loop
     * zero rather than a VelocityVoltage(0) hold.
     */
    private void hardZeroSpindle() {
        m_spindleTargetRps = 0.0;
        m_spindle.set(0.0);
    }

    /** Rev drums AND run spindle together in one command (single requirement). */
    public Command revAndFeed(double rpm, double rps) {
        return runEnd(() -> {
            setDrumRpm(rpm);
            setSpindleRps(rps);
        }, () -> {
            hardZeroDrums();
            hardZeroSpindle();
        });
    }

    /**
     * True when the drums are inside the asymmetric at-speed window of a
     * nonzero target (allowed to be faster than allowed to be slow).
     */
    public boolean drumAtSpeed() {
        if (m_drumTargetRpm == 0.0) {
            return false;
        }
        double leftRpm = Math.abs(m_drumLeft.getVelocity().getValueAsDouble()) * 60.0
            / Constants.Shooter.kDrumGearReduction;
        double rightRpm = Math.abs(m_drumRight.getVelocity().getValueAsDouble()) * 60.0
            / Constants.Shooter.kDrumGearReduction;
        return withinDrumWindow(m_drumTargetRpm, (leftRpm + rightRpm) / 2.0);
    }

    /** Asymmetric at-speed check: faster is OK by more than slower. */
    private static boolean withinDrumWindow(double targetRpm, double actualRpm) {
        double err = targetRpm - actualRpm;
        return err < Constants.Shooter.kDrumUnderTolRpm
            && err > -Constants.Shooter.kDrumOverTolRpm;
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
    public Command shootAtRpm(DoubleSupplier rpm, double spindleShootRps, double spindleIdleRps,
            frc.robot.subsystems.Hood hood) {
        return runEnd(() -> {
            setDrumRpm(rpm.getAsDouble());
            boolean ready = feedReady() && hood.hoodAtTarget();
            SmartDashboard.putBoolean("Shooter/FeedGate", ready);
            // Gate components, so a blocked feed is diagnosable at a glance
            // (1678-style per-condition logging).
            SmartDashboard.putBoolean("Shooter/FeedGate_Drum", m_feedLatched);
            SmartDashboard.putBoolean("Shooter/FeedGate_Hood",
                m_drumTargetRpm != 0.0 && hood.hoodAtTarget());
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
        double drumLeftRpm = Math.abs(m_drumLeft.getVelocity().getValueAsDouble()) * 60.0
            / Constants.Shooter.kDrumGearReduction;
        double drumRightRpm = Math.abs(m_drumRight.getVelocity().getValueAsDouble()) * 60.0
            / Constants.Shooter.kDrumGearReduction;
        double drumAvgRpm = (drumLeftRpm + drumRightRpm) / 2.0;
        double spindleRps = m_spindle.getVelocity().getValueAsDouble()
            / Constants.Shooter.kSpindleReduction;

        SmartDashboard.putNumber("Shooter/DrumRPM", drumAvgRpm);
        SmartDashboard.putNumber("Shooter/DrumTargetRPM", m_drumTargetRpm);
        SmartDashboard.putNumber("Shooter/DrumOffsetRPM", Constants.Shooter.kDrumRpmOffset);
        SmartDashboard.putNumber("Shooter/DrumErrorRPM", m_drumTargetRpm - drumAvgRpm);
        SmartDashboard.putBoolean("Shooter/DrumAtSpeed",
            m_drumTargetRpm != 0.0
                && withinDrumWindow(m_drumTargetRpm, drumAvgRpm));
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
        // Leader output in magnitude: the drum only drives one way (negative
        // sign), so abs keeps the tuning graphs on a 0..12 V / 0..1 axis.
        // These two are the sag-vs-PID tell: duty near 1.0 with error left
        // means out of voltage, low duty with error means FF/tuning.
        SmartDashboard.putNumber("Shooter/DrumVoltage",
            Math.abs(m_drumLeft.getMotorVoltage().getValueAsDouble()));
        SmartDashboard.putNumber("Shooter/DrumDuty",
            Math.abs(m_drumLeft.getDutyCycle().getValueAsDouble()));
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