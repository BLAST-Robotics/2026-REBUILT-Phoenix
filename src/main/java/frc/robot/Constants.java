package frc.robot;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;

public final class Constants {

    public static final String CANIVORE = "canivore";
    public static final String RIO_BUS = "rio";
    public static final CANBus kCANivoreBus = new CANBus(RIO_BUS);

    /** When true, the Control Mode chooser boots into the Test control set. */
    public static final boolean kTestControlsDefaultOnBoot = true;

    public static final class Intake {
        public static final double kPivotGearboxReduction = (5.0 * 3.0 * 3.0); // 45
        public static final double kPivotChainReduction = (36.0 / 12.0);
        public static final double kPivotTotalReduction = kPivotGearboxReduction * kPivotChainReduction;

        public static final double kRollerDiameter = Units.inchesToMeters(1.125);
        public static final double kRollerCircumference = kRollerDiameter * Math.PI;
        public static final double kRollerMaxRPS = 60.0;

        // Pivot axis is on top, inline with the front module axle.
        // Measured on-robot 10/10 (flexible hard stops: readings sit ~0.5 deg
        // inside the absolute max, soft limits pad back out by that 0.5 deg).
        public static final double kPivotStowedRot = -0.0022;   // motion target: stowed
        public static final double kPivotDeployedRot = 0.0054;  // motion target: deployed (roller down)
        public static final double kPivotMinRotations = kPivotStowedRot - 0.5 / 360.0; // soft limit
        public static final double kPivotMaxRotations = kPivotDeployedRot + 0.5 / 360.0; // soft limit
        public static final boolean kPivotInverted = false;
        public static final boolean kRollerInverted = false;

        public static final double kPivotKP = 12.0;
        public static final double kPivotKI = 0.0;
        public static final double kPivotKD = 0.0;
        public static final double kPivotKS = 0.05;

        public static final double kRollerSupplyLimit = 40.0;
        public static final double kPivotSupplyLimit = 40.0;
        public static final double kPivotStatorLimit = 60.0;
        public static final double kPivotVoltageComp = 12.0;

        // Soft limits are compared against mechanism rotations (FusedCANcoder
        // position), so these are raw output rotations with NO reduction scaling.
        public static final double kPivotForwardSoftLimitRot = kPivotMaxRotations;
        public static final double kPivotReverseSoftLimitRot = kPivotMinRotations;

        // Bench-calibrated magnet offset (output rotations). 0.0 = uncalibrated.
        // Datum: fully deployed (roller down). See calibration procedure.
        public static final double kPivotEncoderOffset = 0.0;
    }

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

    public static final class Shooter {
        public static final double kDrumGearReduction = 24.0 / 15.0;
        public static final double kDrumDiameter = Units.inchesToMeters(3.0);
        public static final double kDrumCircumference = kDrumDiameter * Math.PI;
        public static final double kDrumMaxRPM = 6000.0;

        public static final double kSpindleReduction = 24.0 / 15.0;
        public static final double kSpindleDiameter = Units.inchesToMeters(1.125);
        public static final double kSpindleCircumference = kSpindleDiameter * Math.PI;
        public static final double kSpindleMaxRPS = 40.0;

        public static final double kDrumKP = 0.1;
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
        public static final double kHoodTotalReduction = kDriveStage1 * kDriveStage2 * kDriveStage3;

        // Measured on-robot (relative encoder, zeros at boot):
        // -1 deg = minimum, -21 deg = maximum.
        public static final double kHoodMinDegrees = -21.0;
        public static final double kHoodMaxDegrees = -1.0;

        // Bench-calibrated magnet offset (output rotations). 0.0 = uncalibrated.
        // Datum: hood at minimum (-21 deg). See calibration procedure.
        public static final double kHoodEncoderOffset = 0.0;
        public static final double kHoodMinDeg2Rad = Units.degreesToRadians(kHoodMinDegrees);
        public static final double kHoodMaxDegree2Rad = Units.degreesToRadians(kHoodMaxDegrees);

        public static final boolean kHoodMotorInverted = true; // clockwise-positive = up
        public static final boolean kHoodEncoderInverted = false;
        public static final double kHoodDegreesPerRotation = 360.0; // through-bore = 1 rot = 360 deg

        public static final double kHoodKP = 5.0;
        public static final double kHoodKI = 0.0;
        public static final double kHoodKD = 0.0;
        public static final double kHoodKS = 0.05;

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

        // Shooter model: ball pinched between the 3in drum and two 1.125in
        // counter-rollers (drum 40T : roller 24T), with foam slip.
        public static final double kRollerDiameter = Units.inchesToMeters(1.125);
        public static final double kRollerGearRatio = 40.0 / 24.0;
        public static final double kSlip = 0.85;
        public static final double kExitPerDrumRps =
            (Math.PI * Shooter.kDrumDiameter
                + kRollerGearRatio * Math.PI * kRollerDiameter)
                / 2.0 * kSlip;

        // (distance, hood degrees) breakpoints
        public static final double[][] kDistanceToHoodDeg = {
            {0.5, 80.0},
            {1.0, 75.0},
            {1.5, 70.0},
            {2.0, 65.0},
            {2.5, 60.0},
            {3.0, 58.0},
        };
        // (distance, shooter RPM) breakpoints
        public static final double[][] kDistanceToRPM = {
            {0.5, 1765.0},
            {1.0, 1920.0},
            {1.5, 2035.0},
            {2.0, 2135.0},
            {2.5, 2245.0},
            {3.0, 2365.0},
        };
    }

    public static final class Vision {
        public static final String kCameraName = "limelight";
        public static final int kMinTagCount = 1;
        public static final double kMaxPoseError = 0.8;
        public static final String kResolution = "640x480";
        public static final boolean kStream = false;
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

    public static final class old {
        public static final class Intake {
            public static final int kRoller = 40;
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
        public static final int kPigeonId = 15;
    }
}