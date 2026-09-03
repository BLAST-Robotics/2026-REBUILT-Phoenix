package frc.robot;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;

/**
 * Universal + New CANivore Schema constants.
 *
 * <p>This class holds two kinds of values:
 * <ul>
 *   <li><b>Universal constants</b> (top level): mechanism PIDs, feedforwards,
 *       gear ratios, current limits, physical geometry and positions. These are
 *       shared between the old and new schemas - they only change when you tune
 *       the mechanism, not when you migrate CAN wiring.</li>
 *   <li><b>Device IDs</b>: two ID sets are provided.
 *       <ul>
 *         <li>{@link Constants.old} - the EXISTING physical wiring on the robot
 *             today. The code currently reads device numbers from here so it
 *             runs against the current harness.</li>
 *         <li>{@link Constants.ID} - the SUGGESTED clean CANivore schema to
 *             migrate to. Numers are conflict-free with the swerve set and
 *             grouped per mechanism for readability.</li>
 *       </ul>
 *   </li>
 * </ul>
 *
 * <p>To migrate to the new schema: swap {@code Constants.old.X.id} references to
 * {@code Constants.ID.X} and change the CAN bus name to the CANivore. Nothing
 * else changes because the universal constants are shared.
 *
 * <p>NOTE: {@code Constants.old} is a nested class - a separate top-level file
 * cannot be named {@code Constants.old.java} because Java does not allow dots in
 * class/file identifiers. The nested class is the exact {@code Constants.old}
 * namespace requested.
 */
public final class Constants {

    // ==========================================================================
    // CAN BUS NAMES
    // ==========================================================================
    /**
     * Name of the CANivore bus. All CTRE devices (swerve, mechanism motors,
     * CANcoders, Pigeon 2, CANdle LEDs) live here. The PDH 2.0 is the only
     * device on the roboRIO internal bus ({@code "rio"} default).
     */
    public static final String CANIVORE = "canivore";
    /** Default name of the roboRIO internal CAN bus. */
    public static final String RIO_BUS = "rio";
    /** CANBus instance for the CANivore, for the non-deprecated device ctors. */
    public static final CANBus kCANivoreBus = new CANBus(CANIVORE);

    // ==========================================================================
    // INTAKE
    // ==========================================================================
    /**
     * Intake geometry: roller pulls spheres in counter-clockwise positive.
     * Intake pivot: CW = up, CCW = down (motor convention).
     */
    public static final class Intake {
        // --- Gear ratio ---
        // Pivot gearbox: 5:1 x 3:1 x 3:1 planets, then 12T -> 32T chain to the
        // roller. Total reduction = 5*3*3 * (32/12).
        public static final double kPivotGearboxReduction = (5.0 * 3.0 * 3.0); // 45
        public static final double kPivotChainReduction = (32.0 / 12.0);
        public static final double kPivotTotalReduction = kPivotGearboxReduction * kPivotChainReduction;

        // --- Roller ---
        public static final double kRollerDiameter = Units.inchesToMeters(1.125);
        public static final double kRollerCircumference = kRollerDiameter * Math.PI;
        // Motor RPM at full speed -> surface speed. Tune per tuning.
        public static final double kRollerMaxRPS = 60.0;

        // --- Pivot motion limits (positions in rotations of the OUTPUT) ---
        // Geometry: pivot axis is on top, inline with the front module axle.
        // Stowed = arm up over the chassis (kPivotMin). Deployed = arm swung down
        // so the roller is in front of the front modules (kPivotMax).
        public static final double kPivotMinRotations = 0.0;   // stowed (arm up, over chassis)
        public static final double kPivotMaxRotations = 0.15;  // deployed (roller down, in front of modules)
        public static final boolean kPivotInverted = false;
        public static final boolean kRollerInverted = false;

        // --- Pivot closed loop (units: rotations output, controlled via motor
        //     position converted through kPivotTotalReduction) ---
        public static final double kPivotKP = 12.0;
        public static final double kPivotKI = 0.0;
        public static final double kPivotKD = 0.0;
        public static final double kPivotKS = 0.05;

        // --- Limits ---
        public static final double kRollerSupplyLimit = 40.0;
        public static final double kPivotSupplyLimit = 40.0;
        public static final double kPivotStatorLimit = 60.0;
        public static final double kPivotVoltageComp = 12.0;

        // --- Soft limit safety margins (motor rotations after gear ratio) ---
        // This protects the intake from being driven past its hard stops.
        public static final double kPivotForwardSoftLimitRot = kPivotMaxRotations * kPivotTotalReduction;
        public static final double kPivotReverseSoftLimitRot = kPivotMinRotations * kPivotTotalReduction;
    }

    // ==========================================================================
    // SHOPPE : Hopper / Agitator
    // ==========================================================================
    /**
     * Hopper agitator: 16T -> 24T pulley, counter-clockwise pushes spheres
     * forward (toward the shooter).
     */
    public static final class Hopper {
        public static final double kAgitatorReduction = 24.0 / 16.0;
        public static final boolean kAgitatorInverted = false;
        public static final double kAgitatorMaxRPS = 30.0;

        public static final double kAgitatorKP = 0.3;
        public static final double kAgitatorKI = 0.0;
        public static final double kAgitatorKD = 0.0;
        public static final double kAgitatorKS = 0.1;

        public static final double kAgitatorSupplyLimit = 30.0;
        public static final double kAgitatorStatorLimit = 40.0;
    }

    // ==========================================================================
    // SHOOTER
    // ==========================================================================
    /**
     * Shooter: two counter-rotating Kraken x60 drive the drum + flywheels
     * (15:24), plus one Talon X44 spindle drives the internal rollers. The drum
     * passes spheres to the rotating hood rollers (24T->24T pulley, 40T->24T,
     * 24T counter rollers).
     */
    public static final class Shooter {
        // --- Drum ---
        public static final double kDrumGearReduction = 24.0 / 15.0; // 15:24
        public static final double kDrumDiameter = Units.inchesToMeters(3.0);
        public static final double kDrumCircumference = kDrumDiameter * Math.PI;
        // Max drum surface RPM. Tune for the desired shot speed.
        public static final double kDrumMaxRPM = 6000.0;

        // --- Spindle (X44 rollers) ---
        public static final double kSpindleReduction = 24.0 / 15.0;
        public static final double kSpindleDiameter = Units.inchesToMeters(1.125);
        public static final double kSpindleCircumference = kSpindleDiameter * Math.PI;
        public static final double kSpindleMaxRPS = 80.0;

        // --- Drum closed loop (RPM) ---
        public static final double kDrumKP = 0.1;
        public static final double kDrumKI = 0.0;
        public static final double kDrumKD = 0.0;
        public static final double kDrumKV = 0.124;
        public static final double kDrumKA = 0.0;
        // Spindle closed loop (RPS)
        public static final double kSpindleKP = 0.1;
        public static final double kSpindleKI = 0.0;
        public static final double kSpindleKD = 0.0;

        // --- Limits ---
        public static final double kDrumSupplyLimit = 60.0;
        public static final double kDrumStatorLimit = 80.0;
        public static final double kSpindleSupplyLimit = 40.0;
        public static final double kSpindleStatorLimit = 60.0;
    }

    // ==========================================================================
    // HOOD
    // ==========================================================================
    /**
     * Hood: a rotating hood adjusts launch angle. Driven by an X44 through a
     * 15:24 pulley, 30:18 gear, and 10T gear meshing a 153T gear segment on the
     * hood. A WCP through-bore senses the angular position.
     */
    public static final class Hood {
        // Drive reduction from motor to hood output:
        //   15:24 pulley, 30:18 gear, 10T -> 153T gear.
        public static final double kDriveStage1 = 24.0 / 15.0;
        public static final double kDriveStage2 = 30.0 / 18.0;
        public static final double kDriveStage3 = 153.0 / 10.0;
        public static final double kHoodTotalReduction = kDriveStage1 * kDriveStage2 * kDriveStage3;

        // --- Positions (degrees of hood launch angle) ---
        public static final double kHoodMinDegrees = 0.0;
        public static final double kHoodMaxDegrees = 60.0;
        public static final double kHoodMinDeg2Rad = Units.degreesToRadians(kHoodMinDegrees);
        public static final double kHoodMaxDegree2Rad = Units.degreesToRadians(kHoodMaxDegrees);

        public static final boolean kHoodMotorInverted = false;
        public static final boolean kHoodEncoderInverted = false;
        // Encoder to degrees: through-bore absolute -> scale to hood range.
        public static final double kHoodDegreesPerRotation = 360.0; // through-bore = 1 rot = 360 deg

        // --- Closed loop (radians of hood output) ---
        public static final double kHoodKP = 5.0;
        public static final double kHoodKI = 0.0;
        public static final double kHoodKD = 0.0;
        public static final double kHoodKS = 0.05;

        public static final double kHoodSupplyLimit = 40.0;
        public static final double kHoodStatorLimit = 60.0;
    }

    // ==========================================================================
    // AIM (distance -> mechanism setpoints)
    // ==========================================================================
    /**
     * Aiming lookup tables. The Limelight provides field pose; the distance to
     * the target drives the hood angle and shooter RPM through an
     * interpolating tree map. Distances in meters, hood in radians, rpm in RPM.
     */
    public static final class Aim {
        // --- (distance, hood degrees) breakpoints ---
        public static final double[][] kDistanceToHoodDeg = {
            {1.0, 25.0},
            {2.0, 30.0},
            {3.0, 35.0},
            {4.0, 40.0},
            {5.0, 45.0},
            {6.0, 50.0},
        };
        // --- (distance, shooter RPM) breakpoints ---
        public static final double[][] kDistanceToRPM = {
            {1.0, 3000.0},
            {2.0, 4000.0},
            {3.0, 5000.0},
            {4.0, 6000.0},
            {5.0, 6500.0},
            {6.0, 7000.0},
        };
    }

    // ==========================================================================
    // VISION
    // ==========================================================================
    public static final class Vision {
        /** Camera name (matches Limelight config / NT table). */
        public static final String kCameraName = "limelight";
        /** MegaTag2 pose is only trusted with >= this many tags (1 is fine for MT2). */
        public static final int kMinTagCount = 1;
        /** Reject pose estimates distanced farther than this (meters) from odom. */
        public static final double kMaxPoseError = 0.8;

        // See the Elastic layout for the exact tuning values.
        /** CPU bandwidth saver: Megapixel count, lower = faster. */
        public static final String kResolution = "640x480";
        // Whether to send the raw camera stream to the dashboard (off to save
        // bandwidth; LL3 streams over the different NT table anyway).
        public static final boolean kStream = false;
    }

    // ==========================================================================
    // LED
    // ==========================================================================
    public static final class LED {
        /** CTRE CANdle device ID on the CANivore bus. */
        public static final int kCANdleId = 49;
        public static final int kNyLEDCount = 60;
        public static final int kFirstLED = 0;  // 0-7 onboard, 8+ strip
        public static final double kBrightness = 0.5;
    }

    // ==========================================================================
    // DRIVE
    // ==========================================================================
    /** Drivetrain / gyro. IDs are kept from the SWERVE projector (existing). */
    public static final class Drive {
        public static final int kPigeonId = 15;
        // Module positions are defined in TunerConstants (X4 11T, track sizes).
    }

    // ==========================================================================
    // SUGGESTED CANivore ID SCHEMA (target / new)
    // ==========================================================================
    /**
     * New suggested CANivore IDs. Conflict-free with the existing swerve set
     * {1,3,4,6,10,11,13,14,15,23,24,25}. Grouped per mechanism.
     *
     * <p>To use: change the CAN bus to CANivore and reference these instead of
     * {@link Constants.old}.
     */
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

    // ==========================================================================
    // EXISTING / OLD PHYSICAL WIRING (current robot)
    // ==========================================================================
    /**
     * The currently wired hardware. The code reads device IDs from here so it
     * runs against the existing harness. Device numbers within each mechanism
     * mirror {@link Constants.ID} structure; the values reflect the physical
     * CAN IDs in use today (mostly the swerve ids from Tuner X; mechanism ids
     * are placeholders to be confirmed on the real bot).
     *
     * <p>As new mechanism devices are physically wired, update the numbers here.
     */
    public static final class old {
        public static final class Intake {
            public static final int kRoller = 40;     // confirmed 1.125" roller
            public static final int kPivot = 41;
            public static final int kPivotEncoder = 42;
        }
        public static final class Hopper {
            public static final int kAgitator = 43;
        }
        public static final class Shooter {
            public static final int kDrumLeft = 44;
            public static final int kDrumRight = 45;
            public static final int kSpindle = 46;
        }
        public static final class Hood {
            public static final int kMotor = 47;
            public static final int kEncoder = 48;
        }
        public static final class LED {
            public static final int kCANdle = 49;
        }
        // Swerve IDs are kept in TunerConstants (generated). Pigeon for reference:
        public static final int kPigeonId = 15;
    }
}
