package frc.robot;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Maps distance-to-target to hood angle and shooter RPM. */
public final class AimTables {
    private final InterpolatingDoubleTreeMap m_hoodDegrees = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap m_shooterRpm = new InterpolatingDoubleTreeMap();

    private static final AimTables INSTANCE = new AimTables();

    private AimTables() {
        for (double[] bp : Constants.Aim.kDistanceToHoodDeg) {
            m_hoodDegrees.put(bp[0], bp[1]);
        }
        for (double[] bp : Constants.Aim.kDistanceToRPM) {
            m_shooterRpm.put(bp[0], bp[1]);
        }
    }

    public static AimTables getInstance() {
        return INSTANCE;
    }

    /** Hood angle (degrees) for a given distance-to-target (meters). */
    public double hoodDegreesForDistance(double distance) {
        double v = m_hoodDegrees.get(distance);
        SmartDashboard.putNumber("Aim/HoodDeg", v);
        return v;
    }

    /** Shooter RPM for a given distance-to-target (meters). */
    public double shooterRpmForDistance(double distance) {
        double v = m_shooterRpm.get(distance);
        SmartDashboard.putNumber("Aim/DrumRPM", v);
        return v;
    }
}