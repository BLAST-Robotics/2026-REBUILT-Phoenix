package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.Vision;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * Limelight MegaTag1 localization feeding the pose estimator, Team 254 style:
 * MT1 translation fused with gyro heading on single-tag solves (kills the
 * ambiguity flips without MegaTag2's yaw-feed mirror risk), Limelight-side
 * stddevs for trust, timestamp dedup, Z-height sanity.
 *
 * Confident multi-tag solves also fuse vision yaw, so the estimated robot
 * rotation follows the camera. A close, unambiguous single tag may heal a
 * badly wrong gyro heading (>15 deg disagreement).
 *
 * The camera faces straight backwards off the shooter and is not used for
 * direct aiming — pose only.
 */
public class LimelightVision extends SubsystemBase {
    private final CommandSwerveDrivetrain m_drivetrain;
    private final Field2d m_field;
    private final DoubleArrayEntry m_stddevsEntry;
    private double m_lastStamp = -1.0;
    private int m_fusedCount = 0;

    public LimelightVision(CommandSwerveDrivetrain drivetrain, Field2d field) {
        m_drivetrain = drivetrain;
        m_field = field;
        m_stddevsEntry = NetworkTableInstance.getDefault()
            .getTable(Vision.kCameraName).getDoubleArrayTopic("stddevs")
            .getEntry(new double[0]);

        // Single (standard) stream to save CPU/bandwidth.
        LimelightHelpers.setStreamMode_Standard(Vision.kCameraName);
    }

    @Override
    public void periodic() {
        // Match clock + phase for the Driver tab.
        SmartDashboard.putNumber("Match/TimeLeft", DriverStation.getMatchTime());
        String phase = DriverStation.isDisabled() ? "Disabled"
            : DriverStation.isAutonomous() ? "Auto"
            : DriverStation.isTeleop() ? "Teleop" : "Test";
        SmartDashboard.putString("Match/Phase", phase);

        var state = m_drivetrain.getState();        double yawRateDps = Math.toDegrees(state.Speeds.omegaRadiansPerSecond);

        // Docs: 2024+ single-origin — ALWAYS wpiblue, regardless of alliance.
        LimelightHelpers.PoseEstimate estimate =
            LimelightHelpers.getBotPoseEstimate_wpiBlue(Vision.kCameraName);

        SmartDashboard.putBoolean("Vision/HasEstimate", estimate != null);
        if (estimate == null || estimate.tagCount < Vision.kMinTagCount) {
            SmartDashboard.putString("Vision/RejectReason", "no-solve");
            return;
        }
        SmartDashboard.putNumber("Vision/TagCount", estimate.tagCount);
        SmartDashboard.putNumber("Vision/AvgTagDist", estimate.avgTagDist);

        // Timestamp dedup: never fuse the same (or older) solve twice.
        double fpgaNow = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
        double stamp = fpgaNow - estimate.latency / 1000.0;
        if (stamp <= m_lastStamp) {
            SmartDashboard.putString("Vision/RejectReason", "stale-timestamp");
            return;
        }

        // Spinning fast smears solves; drop them.
        if (Math.abs(yawRateDps) > Vision.kMaxYawRateDps) {
            SmartDashboard.putString("Vision/RejectReason", "yaw-rate");
            return;
        }

        // Ambiguity gate on single-tag solves.
        if (estimate.tagCount == 1 && estimate.rawFiducials.length > 0) {
            if (estimate.rawFiducials[0].ambiguity > Vision.kMaxAmbiguity) {
                SmartDashboard.putString("Vision/RejectReason", "ambiguity");
                return;
            }
            // Far single tags are noise (cross-team consensus: 3 m).
            if (estimate.avgTagDist > Vision.kMaxSingleTagDist) {
                SmartDashboard.putString("Vision/RejectReason", "far-single-tag");
                return;
            }
        }

        // Z-height sanity: a solve floating off the floor is garbage.
        double z = LimelightHelpers.getBotPose3d_wpiBlue(Vision.kCameraName).getZ();
        if (Math.abs(z) > Vision.kMaxZMeters) {
            SmartDashboard.putString("Vision/RejectReason", "z-height");
            return;
        }

        Pose2d measured = estimate.pose;

        // Deadlock breaker: booting at the blue corner puts odometry meters
        // from the first true solve, so while disabled a trustworthy solve
        // (multi-tag, or close + unambiguous single) seeds the estimator
        // outright instead of fusing.
        if (DriverStation.isDisabled()) {
            boolean seedTrustworthy = estimate.tagCount >= 2
                || (estimate.avgTagDist < 3.0 && estimate.rawFiducials.length > 0
                    && estimate.rawFiducials[0].ambiguity < Vision.kMaxAmbiguity);
            if (seedTrustworthy) {
                m_drivetrain.resetPose(measured);
                m_lastStamp = stamp;
                SmartDashboard.putString("Vision/RejectReason", "seeded");
                m_field.getObject("vision").setPose(measured);
                return;
            }
        }

        // Field-border gate: solves outside the carpet + margin are garbage.
        double m = Vision.kFieldBorderMargin;
        double x = measured.getX(), y = measured.getY();
        if (x < -m || y < -m
            || x > FieldConstants.kFieldLength + m
            || y > FieldConstants.kFieldWidth + m) {
            SmartDashboard.putString("Vision/RejectReason", "off-field");
            return;
        }

        // Single-tag: keep MT1 translation, substitute gyro heading (254's
        // fuseWithGyro). Kills ambiguity flips. The one exception is the
        // large-disagreement heal below, which trusts a close tag's yaw.
        boolean healSingleTagYaw = false;
        if (estimate.tagCount == 1) {
            measured = new Pose2d(measured.getTranslation(), state.Pose.getRotation());
            if (estimate.rawFiducials.length > 0) {
                double yawDisagreement = Math.abs(estimate.pose.getRotation()
                    .minus(state.Pose.getRotation()).getDegrees());
                boolean healTrustworthy = estimate.avgTagDist < Vision.kSingleTagHealDist
                    && estimate.rawFiducials[0].ambiguity < Vision.kMaxAmbiguity;
                healSingleTagYaw = healTrustworthy
                    && yawDisagreement > Vision.kSingleTagHealMinDeg
                    && yawDisagreement < 175.0;
                if (healSingleTagYaw) {
                    // A close, unambiguous tag disagreeing this much means
                    // the gyro heading is wrong, not the tag: use vision yaw.
                    measured = estimate.pose;
                }
                SmartDashboard.putNumber("Vision/YawDisagreement", yawDisagreement);
            }
        }

        // Don't let vision teleport the robot (threshold currently very high
        // in Constants.Vision.kMaxPoseError — effectively disabled for now).
        double jump = measured.getTranslation().getDistance(
            state.Pose.getTranslation());
        if (jump > Vision.kMaxPoseError) {
            SmartDashboard.putString("Vision/RejectReason", "jump-gate");
            return;
        }

        SmartDashboard.putNumber("Vision/RawX", measured.getX());
        SmartDashboard.putNumber("Vision/RawY", measured.getY());
        SmartDashboard.putNumber("Vision/RawRot", measured.getRotation().getDegrees());

        // Trust: Limelight-side stddevs when present, 0.02*d^2/N fallback —
        // the 3005/3636 consensus (3636 halves it again for MT2, INF yaw).
        double[] sd = m_stddevsEntry.get();
        double xyStd;
        if (sd.length >= 6) {
            xyStd = Math.max(sd[0], sd[1]);
        } else {
            double n = Math.max(1, estimate.tagCount);
            double d = estimate.avgTagDist;
            xyStd = Math.max(0.05, 0.02 * d * d / n);
        }
        // Yaw trust: multi-tag solves have reliable rotation, so fuse it and
        // let the estimator heading follow the camera. Single-tag yaw stays
        // ignored (gyro substitution) unless the heal above fired.
        double yawStd;
        if (estimate.tagCount >= 2) {
            yawStd = Math.toRadians(Vision.kMultiTagYawStdDeg);
        } else if (healSingleTagYaw) {
            yawStd = Math.toRadians(Vision.kSingleTagHealYawStdDeg);
        } else {
            yawStd = 9999999.0;
        }
        Matrix<N3, N1> stddevs = VecBuilder.fill(xyStd, xyStd, yawStd);

        m_lastStamp = stamp;
        m_drivetrain.addVisionMeasurement(measured, stamp, stddevs);
        m_fusedCount++;
        SmartDashboard.putString("Vision/RejectReason", "fused");
        SmartDashboard.putNumber("Vision/FusedCount", m_fusedCount);
        SmartDashboard.putBoolean("Vision/HeadingHeal", healSingleTagYaw);
        SmartDashboard.putBoolean("Vision/YawFused", yawStd < 1.0);

        // Hard snap: normal drift is handled by the fused yaw above, but if a
        // confident multi-tag solve still disagrees badly, the gyro frame
        // itself is wrong. Reset the Pigeon + estimator rotation so aim,
        // pathing, AND field-centric driving all agree with the field.
        boolean yawReset = false;
        if (estimate.tagCount >= 2) {
            double residual = Math.abs(measured.getRotation()
                .minus(state.Pose.getRotation()).getDegrees());
            if (residual > Vision.kYawResetMinDeg) {
                m_drivetrain.resetRotation(measured.getRotation());
                yawReset = true;
                SmartDashboard.putNumber("Vision/YawResetDeg", residual);
            }
        }
        SmartDashboard.putBoolean("Vision/YawReset", yawReset);
        // Draw the accepted vision solve as its own ghost on the shared
        // Elastic field so raw-vs-fused is visible at a glance.
        m_field.getObject("vision").setPose(measured);
        SmartDashboard.putNumber("Vision/LastUpdate", stamp);
        SmartDashboard.putNumber("Vision/MeasAgeMs",
            (fpgaNow - estimate.timestampSeconds) * 1000.0);
    }
}
