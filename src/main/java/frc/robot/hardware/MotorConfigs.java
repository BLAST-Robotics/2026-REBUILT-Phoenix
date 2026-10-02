package frc.robot.hardware;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

/**
 * Shared CTRE (Phoenix 6) motor configuration helpers. Keeping the Phoenix 6
 * config boilerplate in one place so every mechanism uses identical,
 * well-defined behavior (brake idle, voltage compensation, current limits,
 * sensor-to-mechanism ratios, software limits).
 */
public final class MotorConfigs {

    private MotorConfigs() {}

    // ==========================================================================
    // Basic + roller
    // ==========================================================================
    /** Configures a pure percent-voltage roller/flywheel motor. */
    public static void applyRollerConfig(
        TalonFX motor,
        boolean inverted,
        double supplyLimit,
        double statorLimit
    ) {
        TalonFXConfiguration cfg = new TalonFXConfiguration()
            .withSlot0(new Slot0Configs())
            .withMotorOutput(new MotorOutputConfigs()
                .withInverted(inverted
                    ? InvertedValue.Clockwise_Positive
                    : InvertedValue.CounterClockwise_Positive)
.withNeutralMode(NeutralModeValue.Brake))
            .withCurrentLimits(
                new com.ctre.phoenix6.configs.CurrentLimitsConfigs()
                    .withSupplyCurrentLimit(supplyLimit)
                    .withSupplyCurrentLimitEnable(true)
                    .withStatorCurrentLimit(statorLimit)
                    .withStatorCurrentLimitEnable(true));
        motor.getConfigurator().apply(cfg);
    }

    // ==========================================================================
    // Position (pivot-like) with FusedCANcoder + software limits
    // ==========================================================================
    /**
     * Configures a position-controlled motor using a FusedCANcoder on the output
     * shaft, with the reported position in <b>mechanism output rotations</b>.
     *
     * @param totalReduction motor turns per output turn (used for the
     *                       sensor-to-mechanism ratio so controls read output)
     * @param reverseLimitPos  reverse soft limit in output rotations
     * @param forwardLimitPos  forward soft limit in output rotations
     */
    public static void applyPivotConfig(
        TalonFX motor,
        CANcoder encoder,
        boolean motorInverted,
        double supplyLimit,
        double statorLimit,
        double totalReduction,
        double reverseLimitPos,
        double forwardLimitPos
    ) {
        encodeAbsolute(encoder, false);

        TalonFXConfiguration cfg = new TalonFXConfiguration()
            .withSlot0(new Slot0Configs()
                .withKP(12.0).withKI(0).withKD(0).withKS(0.05)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign))
            .withMotorOutput(new MotorOutputConfigs()
                .withInverted(motorInverted
                    ? InvertedValue.Clockwise_Positive
                    : InvertedValue.CounterClockwise_Positive)
.withNeutralMode(NeutralModeValue.Brake))
            .withCurrentLimits(
                new com.ctre.phoenix6.configs.CurrentLimitsConfigs()
                    .withSupplyCurrentLimit(supplyLimit)
                    .withSupplyCurrentLimitEnable(true)
                    .withStatorCurrentLimit(statorLimit)
                    .withStatorCurrentLimitEnable(true))
.withFeedback(new FeedbackConfigs()
                .withFusedCANcoder(encoder)
                .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
                .withSensorToMechanismRatio(totalReduction))
            .withMotionMagic(new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(1.0)
                .withMotionMagicAcceleration(4.0))
            .withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
                .withForwardSoftLimitEnable(true)
                .withReverseSoftLimitEnable(true)
                .withForwardSoftLimitThreshold(forwardLimitPos)
                .withReverseSoftLimitThreshold(reverseLimitPos));
        motor.getConfigurator().apply(cfg);
    }

    // ==========================================================================
    // Velocity (flywheel/agitator) with closed-loop
    // ==========================================================================
    /**
     * Configures a velocity-controlled motor. Reported duty/RPS is in motor
     * rotations. For shooter drums the gear ratio should be folded into the
     * setpoint externally, or use rpmToMotor below.
     */
    public static void applyVelocityConfig(
        TalonFX motor,
        boolean inverted,
        double supplyLimit,
        double statorLimit,
        double kp, double ki, double kd, double kv, double ka
    ) {
        TalonFXConfiguration cfg = new TalonFXConfiguration()
            .withSlot0(new Slot0Configs()
                .withKP(kp).withKI(ki).withKD(kd).withKV(kv).withKA(ka)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign))
            .withMotorOutput(new MotorOutputConfigs()
                .withInverted(inverted
                    ? InvertedValue.Clockwise_Positive
                    : InvertedValue.CounterClockwise_Positive)
.withNeutralMode(NeutralModeValue.Brake))
            .withCurrentLimits(
                new com.ctre.phoenix6.configs.CurrentLimitsConfigs()
                    .withSupplyCurrentLimit(supplyLimit)
                    .withSupplyCurrentLimitEnable(true)
                    .withStatorCurrentLimit(statorLimit)
                    .withStatorCurrentLimitEnable(true));
        motor.getConfigurator().apply(cfg);
    }

    /** Makes the absolute CANcoder read 0 at startup (so output 0 == stowed). */
    public static void encodeAbsolute(CANcoder encoder, boolean inverted) {
        encoder.getConfigurator().apply(new CANcoderConfiguration()
            .withMagnetSensor(
                new com.ctre.phoenix6.configs.MagnetSensorConfigs()
                    .withAbsoluteSensorDiscontinuityPoint(0.50)
                    .withSensorDirection(inverted
                        ? SensorDirectionValue.Clockwise_Positive
                        : SensorDirectionValue.CounterClockwise_Positive)));
    }

    /**
     * Converts a mechanism output RPS to motor RPS given a reduction.
     * Useful for velocity setpoints so code works in mechanism units.
     */
    public static double outputToMotorRps(double outputRps, double reduction) {
        return outputRps * reduction;
    }
}
