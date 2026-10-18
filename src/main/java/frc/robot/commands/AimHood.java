package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.AimTables;
import frc.robot.Constants;
import frc.robot.Constants.Aim;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Hood;

/**
 * Aim HOOD only (never spins the shooter). Looks up hood angle and drum RPM
 * for the distance to the current aim point (the hub, or the own-zone pass
 * point when passing) and holds the hood; drum RPM is dashboard only. Hood
 * output is clamped into the boot-relative window.
 */
public class AimHood extends Command {
    private final CommandSwerveDrivetrain m_drivetrain;
    private final Hood m_hood;
    private final AimTables m_aimTables;
    private final Supplier<Translation2d> m_aimPoint;

    public AimHood(
        CommandSwerveDrivetrain drivetrain,
        Hood hood,
        Supplier<Translation2d> aimPoint
    ) {
        m_drivetrain = drivetrain;
        m_hood = hood;
        m_aimTables = AimTables.getInstance();
        m_aimPoint = aimPoint;
        addRequirements(hood);
    }

    @Override
    public void execute() {
        Pose2d pose = m_drivetrain.getState().Pose;

        // Ball exits 10in behind robot center.
        Translation2d muzzle = pose.getTranslation()
            .minus(new Translation2d(Aim.kMuzzleOffsetBack, 0.0).rotateBy(pose.getRotation()));

        // Robot velocity in field frame.
        ChassisSpeeds speeds = m_drivetrain.getState().Speeds;
        Translation2d fieldVel = new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond)
            .rotateBy(pose.getRotation());

        // Aim at the current target, shifted back by how far the robot moves
        // during the flight. The target is the hub, or the own-zone pass point
        // (hub-dodged) when the robot is outside its alliance zone.
        Translation2d hub = m_aimPoint.get();

        double tof = 0.0;
        double distance = 0.0;
        double hoodDeg = 0.0;
        double rpm = 0.0;
        for (int i = 0; i < 3; i++) {
            distance = muzzle.getDistance(hub.minus(fieldVel.times(tof)));
            hoodDeg = m_aimTables.hoodDegreesForDistance(distance);
            rpm = m_aimTables.shooterRpmForDistance(distance);
            double exitSpeed = Aim.kExitPerDrumRps * (rpm / 60.0);
            // Robot velocity along the shot direction adds to (or steals
            // from) exit speed: driving at the hub throws hotter.
            var shotDir = hub.minus(muzzle);
            double dirNorm = Math.max(1e-6, shotDir.getNorm());
            double vAlong = fieldVel.getX() * shotDir.getX() / dirNorm
                + fieldVel.getY() * shotDir.getY() / dirNorm;
            double horizSpeed = exitSpeed * Math.cos(Math.toRadians(hoodDeg)) + vAlong;
            tof = horizSpeed > 1e-6 ? distance / horizSpeed : 0.0;
        }

        // Clamp into the boot-relative window (0 = stowed delta).
        hoodDeg = Math.max(0.0,
            Math.min(Constants.Hood.kHoodTravelDegrees, hoodDeg));

        SmartDashboard.putNumber("Aim/Distance", distance);
        SmartDashboard.putNumber("Aim/HoodDeg", hoodDeg);
        SmartDashboard.putNumber("Aim/DrumRPM", rpm);
        // Bearing to hub relative to robot heading (0 = straight ahead).
        double hubAngleDeg = hub.minus(muzzle).getAngle().minus(pose.getRotation()).getDegrees();
        SmartDashboard.putNumber("Aim/HubAngleDeg", hubAngleDeg);

        m_hood.setHoodDegrees(hoodDeg);
    }

    @Override
    public void end(boolean interrupted) {
        // Release: stop driving so the stow-on-release binding takes over.
        m_hood.setHoodPercentage(0.0);
    }
}
