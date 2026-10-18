package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.hardware.MotorConfigs;

/** Roller that pulls spheres in, pivot that raises/lowers it. */
public class Intake extends SubsystemBase {
    private final TalonFX m_roller;
    private final TalonFX m_pivot;

    private double m_pivotTargetRot = 0.0;
    // Boot-relative datum: captured lazily. Robot must boot STOWED.
    private boolean m_homed = false;
    private double m_zeroRot = 0.0;

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
            -0.6, 0.6, // Talon soft limits parked wide open; code enforces the delta window
            Constants.Intake.kPivotEncoderOffset, false, false);
        // Snappy profile: full speed most of the way, hard late braking.
        m_pivot.getConfigurator().apply(new com.ctre.phoenix6.configs.MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(1.5)
            .withMotionMagicAcceleration(12.0)
            .withMotionMagicJerk(60.0));
        // Rio-bus CAN relief: auxiliaries don't need 100 Hz feedback.
        m_pivot.getPosition().setUpdateFrequency(50);
        m_pivot.getVelocity().setUpdateFrequency(20);
        pivotEncoder.getAbsolutePosition().setUpdateFrequency(50);

        // Elastic reset button: re-capture datum live (stow by hand first).
        SmartDashboard.putData("Reset Intake", rehomeCommand());
    }

/** Positive percent pulls spheres in. */
    public void setRollerPercentage(double percent) {
        m_roller.set(percent);
    }

    private double m_rollerTargetRps = 0.0;

    /** Closed-loop roller velocity (direct RPS). */
    public void setRollerRps(double rps) {
        m_rollerTargetRps = rps;
        m_roller.setControl(new com.ctre.phoenix6.controls.VelocityVoltage(rps));
    }

    /** Percent drive, code-limited to the delta window. */
    public void setPivotPercentage(double percent) {
        double rel = getPivotOutputRotations();
        if ((rel <= -0.02 && percent < 0) || (rel >= Constants.Intake.kPivotTravelRot && percent > 0)) {
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

    /** Move pivot to a DELTA target (0 = boot/stowed) using Motion Magic. */
    public void setPivotTarget(double deltaRotations) {
        ensureHomed();
        deltaRotations = Math.max(-0.02,
            Math.min(Constants.Intake.kPivotTravelRot, deltaRotations));
        m_pivotTargetRot = deltaRotations;
        m_pivot.setControl(new com.ctre.phoenix6.controls.MotionMagicVoltage(
            Rotations.of(m_zeroRot + deltaRotations)));
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
                    setPivotTarget(Constants.Intake.kPivotDeployedDelta);
                } else {
                    double f = t / 4.0;
                    setPivotTarget(Constants.Intake.kPivotDeployedDelta
                        + (Constants.Intake.kPivotStowedDelta - Constants.Intake.kPivotDeployedDelta) * f);
                }
                setRollerPercentage(0.0);
            }).withTimeout(4.3))
            // Tail: park at deployed; the latch trigger resumes rollers.
            .andThen(runOnce(() -> setPivotTarget(Constants.Intake.kPivotDeployedDelta)));
    }

    /** Capture boot pose as delta zero. Must boot STOWED. */
    private void ensureHomed() {
        if (!m_homed) {
            m_zeroRot = m_pivot.getPosition().getValueAsDouble();
            m_homed = true;
        }
    }

    /** Re-capture the current pose as delta zero (no power cycle needed). */
    public Command rehomeCommand() {
        return runOnce(() -> {
            m_zeroRot = m_pivot.getPosition().getValueAsDouble();
            m_pivotTargetRot = 0.0;
            m_homed = true;
        });
    }

    /** Pivot position in DELTA rotations (0 = boot/stowed pose). */
    public double getPivotOutputRotations() {
        ensureHomed();
        return m_pivot.getPosition().getValueAsDouble() - m_zeroRot;
    }

    public boolean atPivotTarget(double outputRotations, double tolerance) {
        return Math.abs(getPivotOutputRotations() - outputRotations) < tolerance;
    }

    public Command deploy() {
        return runOnce(() -> setPivotTarget(Constants.Intake.kPivotDeployedDelta))
            .andThen(runEnd(() -> setRollerRps(20.0),
                () -> setRollerPercentage(0.0)));
    }

    public Command store() {
        return runOnce(() -> setPivotTarget(Constants.Intake.kPivotStowedDelta))
            .andThen(runEnd(() -> setRollerPercentage(0.0), () -> setRollerPercentage(0.0)));
    }

    /** Roller velocity in RPS (flat 20 everywhere); negative = duty reverse. */
    public Command runRoller(double rps) {
        return run(() -> {
            if (rps <= 0.0) {
                setRollerPercentage(rps);
            } else {
                setRollerRps(rps);
            }
        });
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

    @Override
    public void periodic() {
        double pivotRot = getPivotOutputRotations();
        SmartDashboard.putNumber("Intake/PivotRot", pivotRot);
        SmartDashboard.putNumber("Intake/PivotTargetRot", m_pivotTargetRot);
        SmartDashboard.putBoolean("Intake/PivotAtTarget",
            Math.abs(pivotRot - m_pivotTargetRot) < 0.01);
        SmartDashboard.putBoolean("Intake/Deployed",
            Math.abs(pivotRot - Constants.Intake.kPivotDeployedDelta) < 0.05);
        SmartDashboard.putNumber("Intake/RollCur", m_roller.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Intake/RollSupplyCur", m_roller.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Intake/RollRPS", m_roller.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Intake/RollTargetRPS", m_rollerTargetRps);
        SmartDashboard.putNumber("Intake/RollOut", m_roller.getDutyCycle().getValueAsDouble());
        SmartDashboard.putNumber("Intake/PivotCur", m_pivot.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Intake/PivotSupplyCur", m_pivot.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Intake/PivotTempC", m_pivot.getDeviceTemp().getValueAsDouble());
        SmartDashboard.putBoolean("Intake/Homed", m_homed);
        var pivotFault = m_pivot.getFault_ReverseSoftLimit().getValue()
            || m_pivot.getFault_ForwardSoftLimit().getValue();
        SmartDashboard.putBoolean("Intake/PivotSoftLimitFault", pivotFault);
    }
}
