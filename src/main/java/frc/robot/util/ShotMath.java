package frc.robot.util;

import edu.wpi.first.math.geometry.Translation2d;

import frc.robot.AimTables;
import frc.robot.Constants.Aim;

/**
 * The ONE copy of the aim math: iteratively solve time of flight to the
 * target and look up hood angle + drum RPM from the distance tables.
 * Includes the robot-velocity-along-shot term so driving at the target
 * answers hotter (shoot-on-the-move).
 */
public final class ShotMath {

    /** One aim solution: muzzle-to-target distance, hood angle, drum RPM. */
    public record Shot(double distance, double hoodDeg, double rpm) {}

    private ShotMath() {}

    /** Muzzle point: ball exits 10in behind robot center. */
    public static Translation2d muzzle(Translation2d robotTranslation,
            edu.wpi.first.math.geometry.Rotation2d heading) {
        return robotTranslation
            .minus(new Translation2d(Aim.kMuzzleOffsetBack, 0.0).rotateBy(heading));
    }

    /**
     * Solve hood + RPM for a muzzle point, robot field velocity, and target.
     * 3 fixed-point iterations of the time-of-flight loop (converges fast).
     */
    public static Shot solve(Translation2d muzzle, Translation2d fieldVel, Translation2d target) {
        double tof = 0.0, distance = 0.0, hoodDeg = 0.0, rpm = 0.0;
        for (int i = 0; i < 3; i++) {
            distance = muzzle.getDistance(target.minus(fieldVel.times(tof)));
            hoodDeg = AimTables.getInstance().hoodDegreesForDistance(distance);
            rpm = AimTables.getInstance().shooterRpmForDistance(distance);
            double exitSpeed = Aim.kExitPerDrumRps * (rpm / 60.0);
            // Robot velocity along the shot direction adds to (or steals
            // from) exit speed: driving at the hub throws hotter.
            var shotDir = target.minus(muzzle);
            double dirNorm = Math.max(1e-6, shotDir.getNorm());
            double vAlong = fieldVel.getX() * shotDir.getX() / dirNorm
                + fieldVel.getY() * shotDir.getY() / dirNorm;
            double horizSpeed = exitSpeed * Math.cos(Math.toRadians(hoodDeg)) + vAlong;
            tof = horizSpeed > 1e-6 ? distance / horizSpeed : 0.0;
        }
        return new Shot(distance, hoodDeg, rpm);
    }
}
