package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

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
    private boolean m_holding = false;
    private boolean m_hasTarget = false;
    private double m_lastPct = 0.0;
    private double m_smoothRot = 0.0;
    // Boot-relative datum: captured lazily from the first reading. The robot
    // must boot with the hood STOWED; that pose becomes delta zero.
    private boolean m_homed = false;
    private double m_zeroRot = 0.0;

    public Hood() {
        m_motor = new TalonFX(Constants.ID.kHoodMotor, Constants.kCANBus);
        m_encoder = new CANcoder(Constants.ID.kHoodEncoder, Constants.kCANBus);

        // Motor + sensor BOTH clockwise-positive (flipped in config, no code
        // signs): +output drives deploy-ward with +readings.
        MotorConfigs.applyPivotConfig(m_motor, m_encoder,
            InvertedValue.Clockwise_Positive,
            Constants.Hood.kHoodSupplyLimit, Constants.Hood.kHoodStatorLimit,
            Constants.Hood.kHoodTotalReduction, 1.0, // CANcoder on motor shaft
            -0.5, 0.5, // Talon soft limits parked wide open; code enforces the delta window
            Constants.Hood.kHoodEncoderOffset, true, true);
        // Full PID + gravity hold. Gains live in Constants.Hood so the bench
        // tune doesn't require touching code. Brake neutral mode comes from
        // applyPivotConfig (all mechanism motors brake when idle).
        m_motor.getConfigurator().apply(new com.ctre.phoenix6.configs.Slot0Configs()
            .withKP(Constants.Hood.kHoodKP)
            .withKI(Constants.Hood.kHoodKI)
            .withKD(Constants.Hood.kHoodKD)
            .withKS(Constants.Hood.kHoodKS)
            .withKV(Constants.Hood.kHoodKV)
            .withKG(Constants.Hood.kHoodKG)
            .withGravityType(com.ctre.phoenix6.signals.GravityTypeValue.Elevator_Static)
            .withStaticFeedforwardSign(
                com.ctre.phoenix6.signals.StaticFeedforwardSignValue.UseVelocitySign));
        // Physics cap: motor tops ~100 rot/s ÷ 29.6 ≈ 3.4 mech rot/s.
        // Demanding cruise 10 saturated the output permanently (bang-bang
        // stutter + crawl). A trackable profile is faster in practice.
        m_motor.getConfigurator().apply(new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(2.5)
            .withMotionMagicAcceleration(15.0)
            .withMotionMagicJerk(60.0));
        // Rio-bus CAN relief: auxiliaries don't need 100 Hz feedback.
        m_motor.getPosition().setUpdateFrequency(50);
        m_motor.getVelocity().setUpdateFrequency(20);
        m_encoder.getAbsolutePosition().setUpdateFrequency(50);

        // Elastic reset button: re-capture datum live (stow by hand first).
        SmartDashboard.putData("Reset Hood", rehomeCommand());
    }

    /**
     * Drive to a delta angle (0 = boot/stowed pose, + = deploy-ward) with
     * Talon MotionMagic. The code-P layers are gone: they were scaffolding
     * around CAN starvation (PDH @50 Hz + full-rate swerve aux), since fixed
     * at the source — and the pivot proves the hardware loop flies on this
     * bus. Target = boot datum + delta; clamped centrally.
     */
    public void setHoodDegrees(double degrees) {
        if (!Constants.Enabled.kHood) {
            m_motor.set(0.0);
            return;
        }
        ensureHomed();
        degrees = Math.max(0.0, Math.min(Constants.Hood.kHoodTravelDegrees, degrees));
        // Skip sub-deadband target changes: auto-aim recomputes every loop
        // and pose noise would otherwise re-seed MotionMagic continuously.
        if (m_hasTarget
            && Math.abs(degrees - m_targetDegrees) < Constants.Hood.kHoodTargetDeadbandDeg) {
            return;
        }
        m_targetDegrees = degrees;
        m_hasTarget = true;
        m_motor.setControl(new MotionMagicVoltage(
            Rotations.of(m_zeroRot + degrees / Constants.Hood.kHoodDegreesPerRotation)));
    }

    /** Re-capture the current pose as delta zero (no power cycle needed). */
    public Command rehomeCommand() {
        return runOnce(() -> {
            m_zeroRot = m_motor.getPosition().getValueAsDouble();
            m_smoothRot = 0.0;
            m_targetDegrees = 0.0;
            m_holding = false;
            m_hasTarget = false;
            m_lastPct = 0.0;
            m_homed = true;
        });
    }

    /** Capture boot pose as delta zero. Must boot STOWED. */
    private void ensureHomed() {
        if (!m_homed) {
            m_zeroRot = m_motor.getPosition().getValueAsDouble();
            m_smoothRot = 0.0;
            m_homed = true;
        }
    }

    /**
     * Manual percent jog (HOLD). Capped at the travel stops: no output past
     * 0 deg (stow) or kHoodTravelDegrees (full deploy) — a held stick can't
     * grind the drive into the mechanical stops.
     */
    public void setHoodPercentage(double percent) {
        if (!Constants.Enabled.kHood) {
            percent = 0.0;
        } else if (percent != 0.0) {
            double deg = getHoodDegrees();
            if ((percent > 0.0 && deg >= Constants.Hood.kHoodTravelDegrees)
                || (percent < 0.0 && deg <= 0.0)) {
                percent = 0.0;
            }
        }
        // Manual jog abandons the closed-loop target; make telemetry honest
        // (TargetDeg = now) so the error readout stays meaningful.
        m_hasTarget = false;
        m_targetDegrees = getHoodDegrees();
        m_motor.set(percent);
    }

    /**
     * Disabled-state hold: set coast once, then re-assert zero output every
     * loop so a persisting MotionMagic request can't keep holding position.
     */
    public Command coastHold() {
        return runOnce(() -> setBrakeMode(false))
            .andThen(run(() -> setHoodPercentage(0.0)));
    }

    /** Brake (true) or coast (false) the hood pivot on demand. */
    public void setBrakeMode(boolean brake) {
        m_motor.setNeutralMode(brake
            ? com.ctre.phoenix6.signals.NeutralModeValue.Brake
            : com.ctre.phoenix6.signals.NeutralModeValue.Coast);
    }

    /** Current hood angle in DELTA degrees (0 = boot/stowed pose). */
    public double getHoodDegrees() {
        ensureHomed();
        return (m_motor.getPosition().getValueAsDouble() - m_zeroRot) * Constants.Hood.kHoodDegreesPerRotation;
    }

    public boolean atSetpoint(double degrees, double tolerance) {
        return Math.abs(getHoodDegrees() - degrees) < tolerance;
    }

    /** True when holding the current target (arrival band). */
    public boolean hoodAtTarget() {
        return atSetpoint(m_targetDegrees, 3.0);
    }

    /** Hold-to-run to an angle (same shape as the working manual jog). */
    public Command setAngle(double degrees) {
        return runEnd(() -> setHoodDegrees(degrees), () -> setHoodPercentage(0.0));
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
        SmartDashboard.putBoolean("Hood/Homed", m_homed);
        var hoodFault = m_motor.getFault_ReverseSoftLimit().getValue()
            || m_motor.getFault_ForwardSoftLimit().getValue();
        SmartDashboard.putBoolean("Hood/SoftLimitFault", hoodFault);
    }
}