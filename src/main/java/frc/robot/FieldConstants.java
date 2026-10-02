package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/** Basic field positions for the 2026 REBUILT field. Origin: blue origin. */
public final class FieldConstants {
    public static final double kFieldLength = 16.54175;
    public static final double kFieldWidth = 8.0137;

    // Hub is centered between two Bumps, 158.6in (~4.03m) from the alliance wall.
    public static final double kHubDistFromWall = Units.inchesToMeters(158.6);
    public static final double kHubY = kFieldWidth / 2.0;

    public static final Translation2d kHubBlue = new Translation2d(kHubDistFromWall, kHubY);
    public static final Translation2d kHubRed =
        new Translation2d(kFieldLength - kHubDistFromWall, kHubY);

    private FieldConstants() {}
}