package frc.robot;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;

public final class Constants {

    public static final String RIO_BUS = "rio";
    public static final CANBus kCANBus = new CANBus(RIO_BUS);

    /** When true, the Control Mode chooser boots into the Test control set. */
    public static final boolean kTestControlsDefaultOnBoot = false;

    public static final class Intake {
        public static final double kPivotGearboxReduction = (5.0 * 3.0 * 3.0); // 45
        public static final double kPivotChainReduction = (36.0 / 12.0);
        public static final double kPivotTotalReduction = kPivotGearboxReduction * kPivotChainReduction;

        public static final double kRollerDiameter = Units.inchesToMeters(1.125);
        public static final double kRollerCircumference = kRollerDiameter * Math.PI;
        public static final double kRollerMaxRPS = 60.0;

        // BOOT-RELATIVE frame: the robot must boot with the pivot stowed.
        // At boot the current sensor reading is captured as zero and every
        // target is a delta from there — immune to magnet moves, remounts,
        // and wrap. Stow = 0, deploy = +0.96 (0.95 measured span + margin).
        public static final double kPivotStowedDelta = -0.02;
        public static final double kPivotDeployedDelta = 0.94;
        // Code-enforced window upper bound. Derived from the deploy delta so
        // the deploy target is always reachable (a hardcoded window smaller
        // than the delta would silently clamp deploy short).
        public static final double kPivotTravelRot = kPivotDeployedDelta;
        public static final boolean kPivotInverted = false;
        public static final boolean kRollerInverted = false;

        public static final double kPivotKP = 12.0;
        public static final double kPivotKI = 0.0;
        public static final double kPivotKD = 0.0;
        public static final double kPivotKS = 0.05;

        public static final double kRollerSupplyLimit = 40.0;
        public static final double kRollerStatorLimit = 30.0;
        // Closed-loop roller velocity, flat 20 RPS everywhere.
        public static final double kRollerBaseRPS = 20.0;
        public static final double kRollerKP = 0.3;
        public static final double kRollerKI = 0.0;
        public static final double kRollerKD = 0.0;
        public static final double kRollerKV = 0.12;
        public static final double kRollerKA = 0.0;
        public static final double kPivotSupplyLimit = 40.0;
        public static final double kPivotStatorLimit = 60.0;
        public static final double kPivotVoltageComp = 12.0;

        // Talon soft limits off: the frame is boot-relative (captured at
        // construction), so magnet-frame Talon limits are meaningless.
        // Limits are enforced in code against the delta instead.
        public static final double kPivotForwardSoftLimitRot = 0.0;
        public static final double kPivotReverseSoftLimitRot = 0.0;

        // Bench-calibrated magnet offset (output rotations). 0.0 = uncalibrated.
        // Datum: fully deployed (roller down). See calibration procedure.
        public static final double kPivotEncoderOffset = 0.0;
    }

    public static final class Hopper {
        public static final double kAgitatorReduction = 24.0 / 16.0;
        public static final boolean kAgitatorInverted = false;
        public static final double kAgitatorMaxRPS = 40.0;
        // Match idle trickle (doubled 10/14); spindle runs + (flipped).
        public static final double kAgitatorIdleRPS = 10.0;

        public static final double kAgitatorKP = 0.3;
        public static final double kAgitatorKI = 0.0;
        public static final double kAgitatorKD = 0.0;
        public static final double kAgitatorKS = 0.1;

        public static final double kAgitatorSupplyLimit = 30.0;
        public static final double kAgitatorStatorLimit = 40.0;
    }

    public static final class Shooter {
        // Motor rotations per drum rotation. Flipped 10/17 (was 24/15): the
        // drum is overdriven, so the motor turns fewer times than the drum.
        public static final double kDrumGearReduction = 15.0 / 24.0;
        public static final double kDrumDiameter = Units.inchesToMeters(3.0);
        public static final double kDrumCircumference = kDrumDiameter * Math.PI;
        // Measured 10/14 ceiling on-robot. Targets must stay under this
        // with margin or the up-to-speed gate never opens.
        public static final double kDrumMaxRPM = 3600.0;
        // Full-range drum spool time: 3300 RPM in 0.5 s.
        public static final double kDrumRampRpmPerSec = 6600.0;

        public static final double kSpindleReduction = 24.0 / 15.0;
        public static final double kSpindleDiameter = Units.inchesToMeters(1.125);
        public static final double kSpindleCircumference = kSpindleDiameter * Math.PI;
        public static final double kSpindleMaxRPS = 50.0;
        // Match idle trickle (doubled + flipped 10/14).
        public static final double kSpindleIdleRPS = 10.0;

        public static final double kDrumKP = 0.4;
        public static final double kDrumKI = 0.0;
        public static final double kDrumKD = 0.0;
        public static final double kDrumKV = 0.124;
        public static final double kDrumKA = 0.0;
        public static final double kSpindleKP = 0.1;
        public static final double kSpindleKI = 0.0;
        public static final double kSpindleKD = 0.0;

        public static final double kDrumSupplyLimit = 60.0;
        public static final double kDrumStatorLimit = 80.0;
        public static final double kSpindleSupplyLimit = 40.0;
        public static final double kSpindleStatorLimit = 60.0;
    }

    public static final class Hood {
        public static final double kDriveStage1 = 24.0 / 15.0;
        public static final double kDriveStage2 = 32.0 / 16.0;
        public static final double kDriveStage3 = 153.0 / 10.0;
        // Bench-measured 10/10: 770 sensor-deg over ~26 deg of real travel.
        // The stage math (48.96) disagrees, so one tooth count is wrong;
        // trust the measurement. Fix the tooth counts if ever confirmed.
        public static final double kHoodTotalReduction = 770.0 / 26.0;

        // BOOT-RELATIVE frame: the robot must boot with the hood stowed.
        // Boot reading is captured as zero; every target is a delta from
        // there — immune to magnet moves, remounts, and multi-turn wrap.
        // Stow = 0, deploy = +27.5 (measured travel span, degrees).
        public static final double kHoodTravelDegrees = 27.5;

        // Bench-calibrated magnet offset (output rotations). Unused in the
        // boot-relative frame; kept for a future absolute calibration.
        public static final double kHoodEncoderOffset = 0.0;

        // Hood motor + sensor both clockwise-positive (flipped 10/13 in
        // motor config so +output drives deploy-ward with +readings).
        // Direction is passed as an explicit InvertedValue in Hood.
        public static final boolean kHoodEncoderInverted = true;
        public static final double kHoodDegreesPerRotation = 360.0; // through-bore = 1 rot = 360 deg

        public static final double kHoodKP = 25.0;
        public static final double kHoodKI = 0.0;
        public static final double kHoodKD = 0.5;
        public static final double kHoodKS = 0.3;
        public static final double kHoodKV = 3.0;
        // Gravity hold: EXPERIMENT 10/10 — constant +0.3 V is SUSPECTED of
        // pinning B (it assists stow-ward motion and fights deploy-ward).
        // Zeroed until the drift direction is observed; then re-apply with
        // the correct sign.
        public static final double kHoodKG = 0.0;

        public static final double kHoodSupplyLimit = 40.0;
        public static final double kHoodStatorLimit = 60.0;
    }

    public static final class Aim {
        // Field geometry + hub positions live in FieldConstants.
        public static final double kFieldLength = FieldConstants.kFieldLength;
        public static final double kFieldWidth = FieldConstants.kFieldWidth;
        public static final double kHubDistFromWall = FieldConstants.kHubDistFromWall;

        // Muzzle geometry: ball exits 28in off the carpet, 10in behind robot center.
        public static final double kMuzzleHeight = Units.inchesToMeters(28.0);
        public static final double kMuzzleOffsetBack = Units.inchesToMeters(10.0);

        // Target height: front edge of the 41.7in hexagonal hub opening is 72in up.
        public static final double kTargetHeight = Units.inchesToMeters(72.0);

        // Passing: keep the pass line at least this far (m) from the hub
        // center. If the hub sits on the line to the own alliance zone, the
        // aim point is nudged sideways by this much so the ball clears it.
        public static final double kHubKeepOut = 0.9;

        // Shooter model: ball pinched between the 3in drum and two 1.125in
        // counter-rollers (drum 40T : roller 24T), with foam slip.
        public static final double kRollerDiameter = Units.inchesToMeters(1.125);
        public static final double kRollerGearRatio = 40.0 / 24.0;
        public static final double kSlip = 0.85;
        public static final double kExitPerDrumRps =
            (Math.PI * Shooter.kDrumDiameter
                + kRollerGearRatio * Math.PI * kRollerDiameter)
                / 2.0 * kSlip;

        // (distance, hood degrees) breakpoints — BOOT-RELATIVE DELTAS.
        // Bench 10/14: hood fully DOWN (0) at 1.745 m; ramps up beyond.
        // VERIFY remaining points by shooting.
        public static final double[][] kDistanceToHoodDeg = {
            {0.5, 0.0},
            {1.0, 0.0},
            {1.5, 0.0},
            {2.0, 8.4},
            {2.5, 19.0},
            {3.0, 25},
        };
        // (distance, shooter RPM) breakpoints — CAPPED to the measured
        // 3600 RPM drum ceiling with headroom (3300 max) so the up-to-speed
        // gate can actually trigger. Bench 10/14: 1.745 m needs full send,
        // so the curve climbs to ceiling by ~1.75 m and holds. VERIFY rest.
        public static final double[][] kDistanceToRPM = {
            {0.5, 2500.0},
            {1.0, 2707.0},
            {1.5, 3100.0},
            {2.0, 3300.0},
            {2.5, 3600.0},
            {3.0, 3600.0},
        };
    }

    public static final class Vision {
        public static final String kCameraName = "limelight";
        public static final int kMinTagCount = 1;
        public static final double kMaxPoseError = 1.5;
        // Reject updates while spinning fast: MegaTag2 smears under yaw rate.
        // 180 deg/s (125 runs 100; docs example 360/720).
        public static final double kMaxYawRateDps = 180.0;
        // Reject ambiguous single-tag solves (0 = perfect, 1 = garbage).
        // 0.3 matches 3005's proven threshold.
        public static final double kMaxAmbiguity = 0.3;
        // Reject solves floating off the floor (botpose Z, meters).
        public static final double kMaxZMeters = 0.75;
        // Single-tag solves past this distance are noise (3005: 3.0 m).
        public static final double kMaxSingleTagDist = 3.0;
        // Reject solves outside the field + margin (6328/3005: 0.5 m).
        public static final double kFieldBorderMargin = 0.5;
    }

    public static final class LED {
        public static final int kCANdleId = 49;
        public static final int kNyLEDCount = 60;
        public static final int kFirstLED = 0;  // 0-7 onboard, 8+ strip
        public static final double kBrightness = 0.5;
    }

    public static final class Drive {
        public static final int kPigeonId = 15;
    }

    public static final class ID {
        public static final int kIntakeRoller = 40;
        public static final int kIntakePivot = 41;
        public static final int kIntakePivotEncoder = 42;

        public static final int kAgitator = 43;

        public static final int kDrumLeft = 44;
        public static final int kDrumRight = 45;
        public static final int kSpindle = 46;

        public static final int kHoodMotor = 47;
        public static final int kHoodEncoder = 48;

        public static final int kLED = 49; // CANdle
    }
}