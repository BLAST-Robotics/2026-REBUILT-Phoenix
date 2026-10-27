package frc.robot;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;

public final class Constants {

    public static final String RIO_BUS = "rio";
    public static final CANBus kCANBus = new CANBus(RIO_BUS);

    /** When true, the Control Mode chooser boots into the Test control set. */
    public static final boolean kTestControlsDefaultOnBoot = false;

    /**
     * Master mechanism switches. Flip a value to false to disable that
     * mechanism: every output request is forced to zero, so its bindings,
     * commands, and auto named commands become no-ops. The subsystem is still
     * constructed (safe to leave wired); use this when a mechanism is broken,
     * missing, or being worked on.
     */
    public static final class Enabled {
        public static final boolean kIntake = true;
        public static final boolean kHopper = true;
        public static final boolean kShooter = true;
        public static final boolean kHood = true;
        public static final boolean kLED = true;
    }

    public static final class Intake {
        public static final double kPivotGearboxReduction = (5.0 * 3.0 * 3.0); // 45
        public static final double kPivotChainReduction = (36.0 / 12.0);
        public static final double kPivotTotalReduction = kPivotGearboxReduction * kPivotChainReduction;

        public static final double kRollerDiameter = Units.inchesToMeters(1.125);

        // ABSOLUTE frame: the CANcoder rides the pivot output (1:1), so its
        // calibrated absolute position IS the pivot coordinate. No boot
        // capture / homing. Ends are the MEASURED physical stops; set
        // kPivotEncoderOffset so stowed reads kPivotStowedRot.
        public static final double kPivotStowedRot = -0.48;   // absolute output rotations (measured)
        public static final double kPivotDeployedRot = 0.45; // absolute output rotations (measured)
        // Code-enforced window (measured stow -> deploy span ~0.925 rot).
        public static final double kPivotTravelRot = kPivotDeployedRot - kPivotStowedRot;

        // Stall-home: when the pivot sits within kPivotHomeTolRot of an end and
        // the motor has been stalled (|stator current| >= kPivotHomeCurrentA)
        // for kPivotHomeTimeS, a VIRTUAL trim is nudged so that end reads its
        // exact constant. The CANcoder magnet offset is untouched; the trim is
        // stored in WPILib Preferences so it persists across power cycles.
        public static final boolean kPivotHomeEnabled = true;
        public static final double kPivotHomeTolRot = 0.05;
        public static final double kPivotHomeCurrentA = 25.0;
        public static final double kPivotHomeTimeS = 0.15;

        // Boot homing: on the first enable, drive the pivot to the NEARER end
        // and home there. Duty used to press into the stop, and the give-up
        // timeout if no stall is seen (reversed sensor, jam, etc).
        public static final boolean kPivotHomeOnBoot = true;
        public static final double kPivotHomeDrivePct = 0.15;
        public static final double kPivotHomeTimeoutS = 3.0;
        // Preferences key holding the persistent virtual home trim (rotations).
        public static final String kPivotHomeOffsetKey = "Intake/PivotHomeOffsetRot";
        public static final boolean kPivotInverted = false;
        public static final boolean kRollerInverted = false;

        public static final double kPivotKP = 20.0;
        public static final double kPivotKI = 0.0;
        public static final double kPivotKD = 0.0;
        public static final double kPivotKS = 0.05;

        public static final double kRollerSupplyLimit = 40.0;
        public static final double kRollerStatorLimit = 30.0;
        // Closed-loop roller velocity, flat 25 RPS everywhere (was 20).
        public static final double kRollerBaseRPS = 38.0;
        public static final double kRollerKP = 0.6;
        public static final double kRollerKI = 0.0;
        public static final double kRollerKD = 0.0;
        public static final double kRollerKV = 0.12;
        public static final double kRollerKA = 0.0;
        public static final double kPivotSupplyLimit = 40.0;
        public static final double kPivotStatorLimit = 50.0;

        // Talon soft limits off: the frame is boot-relative (captured at
        // construction), so magnet-frame Talon limits are meaningless.
        // Limits are enforced in code against the delta instead.

        // Bench-calibrated magnet offset (output rotations). 0.0 = uncalibrated.
        // Set so getAbsolutePosition() reads kPivotStowedRot with the pivot
        // physically STOWED (first-boot procedure). Required by the absolute
        // frame: with 0.0 the reading is arbitrary and targets are unsafe.
        public static final double kPivotEncoderOffset = 0.0;
    }

    public static final class Hopper {
        public static final double kAgitatorReduction = 24.0 / 16.0;
        public static final boolean kAgitatorInverted = false;
        public static final double kAgitatorMaxRPS = 40.0;
        // Match idle trickle (doubled 10/14); spindle runs + (flipped).
        public static final double kAgitatorIdleRPS = -10.0;

        public static final double kAgitatorKP = 0.6;
        public static final double kAgitatorKI = 0.0;
        public static final double kAgitatorKD = 0.0;
        public static final double kAgitatorKS = 0.1;

        public static final double kAgitatorSupplyLimit = 30.0;
        public static final double kAgitatorStatorLimit = 40.0;
    }

    public static final class Shooter {
        // Motor rotations per drum rotation. 10/19: now 24:24 (1:1) — the drum
        // is no longer overdriven; drum RPM equals motor RPM.
        public static final double kDrumGearReduction = 24.0 / 24.0;
        public static final double kDrumDiameter = Units.inchesToMeters(3.0);
        public static final double kDrumCircumference = kDrumDiameter * Math.PI;
        // Measured 10/14 ceiling on-robot. Targets must stay under this
        // with margin or the up-to-speed gate never opens.
        public static final double kDrumMaxRPM = 6000.0;
        // Tuning offset (temp, 10/20): every drum setpoint is shifted by this
        // before the velocity loop so PID can be characterized well below the
        // 1:1 free-speed ceiling. Back to 0.0 for match play (as it is now).
        public static final double kDrumRpmOffset = -300.0;
        // Spool ramp, now done in HARDWARE: these two become MotionMagic
        // acceleration/jerk on the drum leader (motor rps/s and rps/s/s).
        // 150 rps/s^2 = the old 9000 rpm/s code ramp; jerk ~2x accel per 1678.
        public static final double kDrumMMAccelRps2 = 150.0;
        public static final double kDrumMMJerkRps3 = 300.0;
        // Measured-velocity smoothing (s) so RPM noise can't chatter the
        // at-speed gate (1678: 0.1).
        public static final double kDrumVelocityFilterS = 0.1;
        // Static-friction feedforward (V), 1678's real-robot 0.19. Bench-tune:
        // raise if the drum drags a fixed RPM below target under no feed,
        // lower if it overshoots coming up.
        public static final double kDrumKS = 0.19;
        // OVERRAMP (1678 trick): when the drums start from a stop, the target
        // is commanded this much ABOVE the real setpoint and decays to 0 over
        // kDrumOverrampTimeS, so the first fed ball can't sag through the
        // at-speed window.
        public static final double kDrumOverrampRpm = 300.0;
        public static final double kDrumOverrampTimeS = 0.5;
        // "At speed" band for the feed gate, ASYMMETRIC: the drum may run up to
        // kDrumOverTolRpm fast, but only kDrumUnderTolRpm slow, of target.
        public static final double kDrumOverTolRpm = 300.0;
        public static final double kDrumUnderTolRpm = 100.0;

        public static final double kSpindleReduction = 24.0 / 15.0;
        public static final double kSpindleDiameter = Units.inchesToMeters(1.125);
        public static final double kSpindleCircumference = kSpindleDiameter * Math.PI;
        // Match idle trickle (doubled + flipped 10/14).
        public static final double kSpindleIdleRPS = 10.0;

        // Drum drive direction: the drum only ever runs the feeding way, so
        // this is the ONE place the sign lives. Every RPM setpoint in the code
        // is a POSITIVE magnitude, applied here by Shooter.setDrumRpm().
        public static final double kDrumDriveSign = -1.0;

        // Drum velocity loop. KV is the bulk of the feedforward. It is a MOTOR
        // constant, so it must match the Kraken X60 free speed regardless of the
        // drum gearing: 12 V / (5800 rpm / 60) = 0.1241 V per motor rps.
        // (It was 0.17 = a 4235 rpm motor. At 1:1 that saturates the
        // feedforward around 4200 drum rpm, so the loop railed and could not
        // regulate above it — the "4000 and sag" behavior.)
        // 2910-style velocity loop: the feedforward (KV) does the work, so KP
        // only trims. KP 2.0 with a 1:1 drum was a ±500 RPM limit cycle
        // (visible on Shooter/DrumErrorRPM); 0.35-0.4 is the working range.
        public static final double kDrumKP = 0.4;
        public static final double kDrumKI = 0.0;
        public static final double kDrumKD = 0.0;
        public static final double kDrumKV = 12.0 / (5800.0 / 60.0);
        public static final double kDrumKA = 0.0;
        public static final double kSpindleKP = 0.4;
        public static final double kSpindleKI = 0.0;
        public static final double kSpindleKD = 0.0;

        // Raised 10/18 from 60 A to give the drum more headroom to hold RPM
        // under load (it was sagging a few hundred rpm). Watch the main
        // breaker: 2 motors x 80 A = 160 A potential vs the 120 A main, so
        // this is fine for spin-up transients but don't let both sustain it.
        public static final double kDrumSupplyLimit = 80.0;
        public static final double kDrumStatorLimit = 100.0;
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
        public static final double kHoodDegreesPerRotation = 360.0; // through-bore = 1 rot = 360 deg

        // Hold tune (10/19): the static-friction term was flipping sign on
        // noise around zero error; switched to UseVelocitySign and cut KD
        // (derivative on a 50 Hz CANcoder only adds noise). MotionMagic's
        // profile supplies the velocity feedforward, so KD is not needed.
        // 10/20: kS (now 0.25, halved from the 1678-style 0.45 bench start)
        // a middle profile - snappier than the original, less jumpy than
        // the 25/5 sweep.
        public static final double kHoodKP = 30.0;
        public static final double kHoodKI = 0.0;
        public static final double kHoodKD = 0.0;
        public static final double kHoodKV = 5.0;
        // MotionMagic profile (mechanism rot/s through the 29.6:1 winch).
        public static final double kHoodMMCruiseRps = 3.0;
        public static final double kHoodMMAccelRps2 = 18.0;
        public static final double kHoodMMJerkRps3 = 75.0;
        // Ignore aim-target changes smaller than this so pose noise doesn't
        // re-seed MotionMagic every loop (another source of visible buzz).
        public static final double kHoodTargetDeadbandDeg = 0.15;
        // Gravity hold, MEASURED 10/20 by parked-offset math (both runs agree):
        // kG 0 parked the hood -5.5 deg (KP 15) => G = 5.5/360*15 = 0.23 V;
        // kG 0.5 parked +4.6 deg (KP 25) => G = 0.18 V. kS (UseVelocitySign)
        // does nothing at zero speed, so kG is the ONLY static gravity
        // counter. Set to the measured 0.20 V; parked error should sit near
        // zero. Nudge in 0.02 steps if it hangs.
        public static final double kHoodKG = 0.20;
        public static final double kHoodKS = 0.25;

        public static final double kHoodSupplyLimit = 30.0;
        public static final double kHoodStatorLimit = 40.0;
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

        // Shot aim point: 6in from the hub CENTER, toward the middle of the
        // field (the near face of the hub). The pass keep-out still uses the
        // true hub center.
        public static final double kHubAimOffset = Units.inchesToMeters(12.0);

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
        // 10/20: slope reduced ~20% (3->9.5 becomes 3->8.0), the ball with
        // +300 RPM carries some of the flattening itself.
        public static final double[][] kDistanceToHoodDeg = {
                    {0.5, 3.0},
                    {1.0, 3.4},
                    {1.5, 4.6},
                    {2.0, 5.8},
                    {2.5, 7.0},
                    {3.0, 7.8},
                    {3.5, 7.8},
                    {4.0, 8.0},
        };

        public static final double[][] kDistanceToRPM = {
                    {0.5, 3100.0},
                    {1.0, 3350.0},
                    {1.5, 3500.0},
                    {2.0, 3750.0},
                    {2.5, 4000.0},
                    {3.0, 4200.0},
                    {3.5, 4500.0},
                    {4.0, 4750.0},
        };
    }

    public static final class Vision {
        public static final String kCameraName = "limelight";
        public static final int kMinTagCount = 1;
        // Robot-jump gate: reject solves this far (m) from the estimator pose.
        // Set very high to effectively disable it for now (real value was 1.5).
        public static final double kMaxPoseError = 100.0;
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

        // Confident multi-tag solves fuse vision yaw, so the estimator
        // rotation follows the camera (the "update rotation when confident"
        // path). 4 deg is tight enough to hold heading but loose enough not
        // to fight the gyro on every frame.
        public static final double kMultiTagYawStdDeg = 4.0;
        // Single-tag yaw is ambiguous and is normally replaced with the gyro
        // heading. If a CLOSE, unambiguous tag still disagrees by > 15 deg,
        // the gyro heading is the problem: heal it with vision yaw.
        public static final double kSingleTagHealDist = 2.5;
        public static final double kSingleTagHealMinDeg = 15.0;
        public static final double kSingleTagHealYawStdDeg = 10.0;
        // Hard snap: fused yaw handles normal drift. If a confident
        // multi-tag solve still disagrees by more than this, the gyro frame
        // itself is wrong, so resetRotation snaps the Pigeon + estimator.
        public static final double kYawResetMinDeg = 10.0;
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