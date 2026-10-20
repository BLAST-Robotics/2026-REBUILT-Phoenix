package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * Passing geometry for the 2026 REBUILT field.
 *
 * <p>A pass aims the muzzle into the robot's own alliance zone (a blue robot
 * passes into the blue zone). The hub sits between the neutral zone and each
 * alliance zone, so a naive pass line often runs straight through it. When it
 * does, {@link #safeAimPoint} nudges the aim point sideways to clear the hub.
 */
public final class PassGeometry {

    private PassGeometry() {}

    /** The own-alliance pass target (center of that zone). */
    public static Translation2d passTarget(boolean redAlliance) {
        return redAlliance ? FieldConstants.kPassTargetRed : FieldConstants.kPassTargetBlue;
    }

    /**
     * The alliance's scoring hub, with the aim offset applied: 6in from the
     * hub center toward the middle of the field (the near face).
     */
    public static Translation2d hub(boolean redAlliance) {
        Translation2d base = redAlliance ? FieldConstants.kHubRed : FieldConstants.kHubBlue;
        double shift = redAlliance
            ? -Constants.Aim.kHubAimOffset
            : Constants.Aim.kHubAimOffset;
        return new Translation2d(base.getX() + shift, base.getY());
    }

    /**
     * Aim point for a pass launched from {@code from}: the own-zone target,
     * laterally shifted when the hub lies between the robot and the target
     * within the keep-out radius.
     */
    public static Translation2d safeAimPoint(Translation2d from, boolean redAlliance) {
        return avoidObstacle(from, passTarget(redAlliance), hub(redAlliance),
            Constants.Aim.kHubKeepOut);
    }

    /**
     * Shifts {@code target} perpendicular to the from-&gt;target line so the
     * line clears {@code obstacle} by at least {@code keepOut} meters, on the
     * side the obstacle is <em>not</em> on. Returns {@code target} unchanged
     * when the obstacle is not between the endpoints or the line already
     * clears it.
     */
    static Translation2d avoidObstacle(
            Translation2d from, Translation2d target, Translation2d obstacle, double keepOut) {
        Translation2d toTarget = target.minus(from);
        double span = toTarget.getNorm();
        if (span < 1e-6) {
            return target;
        }
        Translation2d dir = toTarget.div(span);
        // Left-hand normal of the from->target direction.
        Translation2d perp = new Translation2d(-dir.getY(), dir.getX());

        Translation2d toObstacle = obstacle.minus(from);
        double along = toObstacle.dot(dir);
        double lateral = toObstacle.dot(perp);

        // Only matters if the hub actually sits between robot and target.
        if (along <= 0.0 || along >= span || Math.abs(lateral) >= keepOut) {
            return target;
        }
        // Push the aim point past the far side of the hub: the line's lateral
        // offset at the hub becomes -sign(lateral) * keepOut.
        double side = lateral == 0.0 ? 1.0 : Math.signum(lateral);
        return target.plus(perp.times(lateral + side * keepOut));
    }
}
