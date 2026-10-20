package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.hardware.MotorConfigs;

/** Roller that pulls spheres in, pivot that raises/lowers it. */
public class Intake extends SubsystemBase {
    private final TalonFX m_roller;
    private final TalonFX m_pivot;

    // ABSOLUTE frame: the CANcoder rides the pivot output (1:1) and its
    // calibrated magnet offset (Constants.Intake.kPivotEncoderOffset) puts the
    // reading in the pivot coordinate. Every target is an absolute output
    // rotation. A RAM-only virtual trim (m_homeOffsetRot) lets the home
    // routines re-zero against a hard stop.
    private double m_pivotTargetRot = Constants.Intake.kPivotStowedRot;

    // Virtual home trim (rotations) added to the raw reading. 0.0 = as
    // calibrated. Persisted in WPILib Preferences (NOT the CANcoder), so it
    // survives power cycles without touching hardware.
    private double m_homeOffsetRot = 0.0;
    private double m_stallTimerS = 0.0;
    private boolean m_homed = false;
    private boolean m_homing = false;

    public Intake() {
m_roller = new TalonFX(Constants.ID.kIntakeRoller, Constants.kCANBus);
        m_pivot = new TalonFX(Constants.ID.kIntakePivot, Constants.kCANBus);
        CANcoder pivotEncoder = new CANcoder(Constants.ID.kIntakePivotEncoder, Constants.kCANBus);

        MotorConfigs.applyVelocityConfig(m_roller,
            Constants.Intake.kRollerInverted,
            Constants.Intake.kRollerSupplyLimit, Constants.Intake.kRollerStatorLimit,
            Constants.Intake.kRollerKP, Constants.Intake.kRollerKI, Constants.Intake.kRollerKD,
            Constants.Intake.kRollerKV, Constants.Intake.kRollerKA);

        // Roller coasts always; only pivots ever brake.
        m_roller.setNeutralMode(com.ctre.phoenix6.signals.NeutralModeValue.Coast);

        MotorConfigs.applyPivotConfig(m_pivot, pivotEncoder,
            Constants.Intake.kPivotInverted
                ? com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive
                : com.ctre.phoenix6.signals.InvertedValue.CounterClockwise_Positive,
            Constants.Intake.kPivotSupplyLimit, Constants.Intake.kPivotStatorLimit,
            1.0, Constants.Intake.kPivotTotalReduction, // CANcoder on output: sensor:mech 1:1
            // Talon soft limits parked on the window but DISABLED: they are
            // only valid once kPivotEncoderOffset is calibrated, so the code
            // enforces the absolute window instead.
            Constants.Intake.kPivotStowedRot - 0.05, Constants.Intake.kPivotDeployedRot + 0.05,
            Constants.Intake.kPivotEncoderOffset, false, false);
        // Pivot loop gains. applyPivotConfig only sets shared defaults, so
        // apply the Constants.Intake gains explicitly here (tunable without
        // touching MotorConfigs).
        m_pivot.getConfigurator().apply(new com.ctre.phoenix6.configs.Slot0Configs()
            .withKP(Constants.Intake.kPivotKP)
            .withKI(Constants.Intake.kPivotKI)
            .withKD(Constants.Intake.kPivotKD)
            .withKS(Constants.Intake.kPivotKS)
            .withStaticFeedforwardSign(
                com.ctre.phoenix6.signals.StaticFeedforwardSignValue.UseClosedLoopSign));
        // Snappy profile: full speed most of the way, hard late braking.
        m_pivot.getConfigurator().apply(new com.ctre.phoenix6.configs.MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(1.5)
            .withMotionMagicAcceleration(12.0)
            .withMotionMagicJerk(60.0));
        // Rio-bus CAN relief: auxiliaries don't need 100 Hz feedback.
        m_pivot.getPosition().setUpdateFrequency(50);
        m_pivot.getVelocity().setUpdateFrequency(20);
        pivotEncoder.getAbsolutePosition().setUpdateFrequency(50);

        // Restore the persistent virtual home trim (written by the home routines).
        m_homeOffsetRot = Preferences.getDouble(Constants.Intake.kPivotHomeOffsetKey, 0.0);

        // Elastic button: forget the stored home (next boot will re-home).
        SmartDashboard.putData("Reset Intake Home", resetHome());
    }

/** Positive percent pulls spheres in. */
    public void setRollerPercentage(double percent) {
        if (!Constants.Enabled.kIntake) {
            m_roller.set(0.0);
            return;
        }
        m_roller.set(percent);
    }

    private double m_rollerTargetRps = 0.0;

    /** Closed-loop roller velocity (direct RPS). */
    public void setRollerRps(double rps) {
        if (!Constants.Enabled.kIntake) {
            m_roller.set(0.0);
            return;
        }
        m_rollerTargetRps = rps;
        m_roller.setControl(new com.ctre.phoenix6.controls.VelocityVoltage(rps));
    }

    /** Percent drive, code-limited to the absolute window. */
    public void setPivotPercentage(double percent) {
        if (!Constants.Enabled.kIntake) {
            m_pivot.set(0.0);
            return;
        }
        double pos = getPivotOutputRotations();
        if ((pos <= Constants.Intake.kPivotStowedRot && percent < 0)
            || (pos >= Constants.Intake.kPivotDeployedRot && percent > 0)) {
            m_pivot.set(0.0);
            return;
        }
        m_pivot.set(percent);
    }

    /** Brake (true) or coast (false) the intake pivot on demand. */
    public void setPivotBrakeMode(boolean brake) {
        m_pivot.setNeutralMode(brake
            ? com.ctre.phoenix6.signals.NeutralModeValue.Brake
            : com.ctre.phoenix6.signals.NeutralModeValue.Coast);
    }

    /** Move pivot to an ABSOLUTE output-rotation target using Motion Magic. */
    public void setPivotTarget(double rotations) {
        if (!Constants.Enabled.kIntake) {
            m_pivot.set(0.0);
            return;
        }
        rotations = Math.max(Constants.Intake.kPivotStowedRot,
            Math.min(Constants.Intake.kPivotDeployedRot, rotations));
        m_pivotTargetRot = rotations;
        m_pivot.setControl(new com.ctre.phoenix6.controls.MotionMagicVoltage(
            Rotations.of(rotations - m_homeOffsetRot)));
    }

    /**
     * 4-second shooting dip: rollers muted, pivots up from deployed toward
     * stowed over 4 s, then re-deploys (rollers resume via deploy()).
     * Trigger on shoot press.
     */
    public Command shootDip() {
        final double[] start = {0.0};
        return runOnce(() -> {
                start[0] = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
                setRollerPercentage(0.0);
            })
            .andThen(run(() -> {
                double t = edu.wpi.first.wpilibj.Timer.getFPGATimestamp() - start[0];
                if (t >= 4.0) {
                    setPivotTarget(Constants.Intake.kPivotDeployedRot);
                } else {
                    double f = t / 4.0;
                    setPivotTarget(Constants.Intake.kPivotDeployedRot
                        + (Constants.Intake.kPivotStowedRot - Constants.Intake.kPivotDeployedRot) * f);
                }
                setRollerPercentage(0.0);
            }).withTimeout(4.3))
            // Tail: park at deployed; the latch trigger resumes rollers.
            .andThen(runOnce(() -> setPivotTarget(Constants.Intake.kPivotDeployedRot)));
    }

    /** Pivot position in the virtual-homed frame (raw reading + home trim). */
    public double getPivotOutputRotations() {
        return m_pivot.getPosition().getValueAsDouble() + m_homeOffsetRot;
    }

    /** True if a persistent home trim is stored (boot homing is then skipped). */
    public boolean hasStoredHome() {
        return Preferences.containsKey(Constants.Intake.kPivotHomeOffsetKey);
    }

    /** Remove the stored home preference and zero the virtual trim. */
    public Command resetHome() {
        return runOnce(() -> {
            Preferences.remove(Constants.Intake.kPivotHomeOffsetKey);
            m_homeOffsetRot = 0.0;
            m_homed = false;
        });
    }

    public boolean atPivotTarget(double outputRotations, double tolerance) {
        return Math.abs(getPivotOutputRotations() - outputRotations) < tolerance;
    }

    public Command deploy() {
        return runOnce(() -> setPivotTarget(Constants.Intake.kPivotDeployedRot))
            .andThen(runEnd(() -> setRollerRps(Constants.Intake.kRollerBaseRPS),
                () -> setRollerPercentage(0.0)));
    }

    public Command store() {
        return runOnce(() -> setPivotTarget(Constants.Intake.kPivotStowedRot))
            .andThen(runEnd(() -> setRollerPercentage(0.0), () -> setRollerPercentage(0.0)));
    }

    /**
     * Roller velocity in RPS (flat 20 everywhere); negative = duty reverse.
     * Ends with the roller off: a bare run() would leave the last closed-loop
     * VelocityVoltage request latched in the Talon, so the roller kept
     * spinning after the command ended (e.g. once the intake stowed).
     */
    public Command runRoller(double rps) {
        return runEnd(() -> {
            if (rps <= 0.0) {
                setRollerPercentage(rps);
            } else {
                setRollerRps(rps);
            }
        }, () -> setRollerPercentage(0.0));
    }

    /** Full neutral: roller off + pivot control neutralized (MotionMagic
     * requests otherwise persist in the Talon across disable/enable). */
    public void killOutputs() {
        setRollerPercentage(0.0);
        m_pivot.set(0.0);
    }

    /**
     * Disabled-state hold: set the pivot to coast once, then re-assert zero
     * outputs every loop. A one-shot on disable is not enough — a MotionMagic
     * request issued before the transition (or a still-running command)
     * otherwise keeps holding position under power, which feels like brake.
     */
    public Command coastHold() {
        return runOnce(() -> setPivotBrakeMode(false))
            .andThen(run(() -> {
                setPivotPercentage(0.0);
                setRollerPercentage(0.0);
            }));
    }

    /**
     * Automatic stall-home. If the pivot sits within kPivotHomeTolRot of an
     * end while the motor is stalled, nudge the virtual trim so that end reads
     * its exact constant. Called every loop from periodic().
     */
    private void checkStallHome() {
        if (m_homing || !Constants.Intake.kPivotHomeEnabled || !Constants.Enabled.kIntake) {
            m_stallTimerS = 0.0;
            return;
        }
        double pos = getPivotOutputRotations();
        double end = 0.0;
        boolean nearEnd = false;
        if (Math.abs(pos - Constants.Intake.kPivotStowedRot) <= Constants.Intake.kPivotHomeTolRot) {
            end = Constants.Intake.kPivotStowedRot;
            nearEnd = true;
        } else if (Math.abs(pos - Constants.Intake.kPivotDeployedRot) <= Constants.Intake.kPivotHomeTolRot) {
            end = Constants.Intake.kPivotDeployedRot;
            nearEnd = true;
        }
        double cur = Math.abs(m_pivot.getStatorCurrent().getValueAsDouble());
        if (nearEnd && cur >= Constants.Intake.kPivotHomeCurrentA) {
            m_stallTimerS += 0.02;
            if (m_stallTimerS >= Constants.Intake.kPivotHomeTimeS) {
                persistHome(end, pos);
                m_stallTimerS = 0.0;
            }
        } else {
            m_stallTimerS = 0.0;
        }
    }

    /**
     * Update the virtual trim so the current physical pose reads {@code endRot},
     * and store it in Preferences so it survives power cycles. The CANcoder
     * magnet offset is never touched. Idempotent once home.
     */
    private void persistHome(double endRot, double measuredRot) {
        m_homeOffsetRot += (endRot - measuredRot);
        Preferences.setDouble(Constants.Intake.kPivotHomeOffsetKey, m_homeOffsetRot);
        m_pivot.set(0.0);          // stop pushing into the hard stop
        m_pivotTargetRot = endRot;
        m_homed = true;
    }

    /**
     * Boot homing: pick the nearer travel end from the current reading, drive
     * into it at kPivotHomeDrivePct until the motor stalls, then set the
     * virtual trim so that end reads its exact constant. Ends parked at that
     * end. Times out (leaving the trim untouched) if no stall is seen — e.g.
     * reversed sensor or a jam.
     */
    public Command homeToNearestEnd() {
        final double[] dir = {0.0};
        final double[] stall = {0.0};
        final boolean[] done = {false};
        return runOnce(() -> {
                double pos = getPivotOutputRotations();
                double dStow = Math.abs(pos - Constants.Intake.kPivotStowedRot);
                double dDeploy = Math.abs(pos - Constants.Intake.kPivotDeployedRot);
                dir[0] = (dStow <= dDeploy) ? -1.0 : 1.0;
                stall[0] = 0.0;
                done[0] = false;
                m_homing = true;
            })
            .andThen(run(() -> {
                m_pivot.set(dir[0] * Constants.Intake.kPivotHomeDrivePct);
                double cur = Math.abs(m_pivot.getStatorCurrent().getValueAsDouble());
                stall[0] = cur >= Constants.Intake.kPivotHomeCurrentA ? stall[0] + 0.02 : 0.0;
                if (stall[0] >= Constants.Intake.kPivotHomeTimeS) {
                    double endRot = dir[0] < 0
                        ? Constants.Intake.kPivotStowedRot
                        : Constants.Intake.kPivotDeployedRot;
                    persistHome(endRot, getPivotOutputRotations());
                    done[0] = true;
                }
            }).until(() -> done[0]))
            .finallyDo(interrupted -> {
                m_pivot.set(0.0);
                m_homing = false;
            })
            .withTimeout(Constants.Intake.kPivotHomeTimeoutS);
    }

    @Override
    public void periodic() {
        checkStallHome();
        double pivotRot = getPivotOutputRotations();
        SmartDashboard.putNumber("Intake/PivotRot", pivotRot);
        SmartDashboard.putNumber("Intake/PivotTargetRot", m_pivotTargetRot);
        SmartDashboard.putBoolean("Intake/PivotAtTarget",
            Math.abs(pivotRot - m_pivotTargetRot) < 0.01);
        SmartDashboard.putBoolean("Intake/Deployed",
            Math.abs(pivotRot - Constants.Intake.kPivotDeployedRot) < 0.05);
        // Reading outside the window means the base calibration is off (or the
        // magnet moved): targets are unsafe until it is corrected.
        SmartDashboard.putBoolean("Intake/PivotAbsInWindow",
            pivotRot >= Constants.Intake.kPivotStowedRot - 0.05
                && pivotRot <= Constants.Intake.kPivotDeployedRot + 0.05);
        SmartDashboard.putNumber("Intake/HomeOffsetRot", m_homeOffsetRot);
        SmartDashboard.putBoolean("Intake/Homed", m_homed);
        SmartDashboard.putBoolean("Intake/Stalling",
            Math.abs(m_pivot.getStatorCurrent().getValueAsDouble()) >= Constants.Intake.kPivotHomeCurrentA);
        SmartDashboard.putNumber("Intake/RollCur", m_roller.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Intake/RollSupplyCur", m_roller.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Intake/RollRPS", m_roller.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Intake/RollTargetRPS", m_rollerTargetRps);
        SmartDashboard.putNumber("Intake/RollOut", m_roller.getDutyCycle().getValueAsDouble());
        SmartDashboard.putNumber("Intake/PivotCur", m_pivot.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Intake/PivotSupplyCur", m_pivot.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Intake/PivotTempC", m_pivot.getDeviceTemp().getValueAsDouble());
        var pivotFault = m_pivot.getFault_ReverseSoftLimit().getValue()
            || m_pivot.getFault_ForwardSoftLimit().getValue();
        SmartDashboard.putBoolean("Intake/PivotSoftLimitFault", pivotFault);
    }
}
