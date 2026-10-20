package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Hood;
import frc.robot.util.ShotMath;

/**
 * Aim HOOD only (never spins the shooter). Looks up hood angle and drum RPM
 * for the distance to the current aim point (the hub, or the own-zone pass
 * point when passing) and holds the hood; drum RPM is dashboard only. Hood
 * output is clamped into the boot-relative window.
 */
public class AimHood extends Command {
    private final CommandSwerveDrivetrain m_drivetrain;
    private final Hood m_hood;
    private final Supplier<Translation2d> m_aimPoint;

    public AimHood(
        CommandSwerveDrivetrain drivetrain,
        Hood hood,
        Supplier<Translation2d> aimPoint
    ) {
        m_drivetrain = drivetrain;
        m_hood = hood;
        m_aimPoint = aimPoint;
        addRequirements(hood);
    }

    @Override
    public void execute() {
        Pose2d pose = m_drivetrain.getState().Pose;

        // Robot velocity in field frame.
        ChassisSpeeds speeds = m_drivetrain.getState().Speeds;
        Translation2d fieldVel = new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond)
            .rotateBy(pose.getRotation());

        // Aim at the current target, shifted back by how far the robot moves
        // during the flight. The target is the hub, or the own-zone pass point
        // (hub-dodged) when the robot is outside its alliance zone.
        Translation2d hub = m_aimPoint.get();

        var shot = ShotMath.solve(ShotMath.muzzle(pose.getTranslation(), pose.getRotation()), fieldVel, hub);
        double distance = shot.distance();
        double hoodDeg = shot.hoodDeg();
        double rpm = shot.rpm();

        // Clamp into the boot-relative window (0 = stowed delta).
        hoodDeg = Math.max(0.0,
            Math.min(Constants.Hood.kHoodTravelDegrees, hoodDeg));

        SmartDashboard.putNumber("Aim/Distance", distance);
        SmartDashboard.putNumber("Aim/HoodDeg", hoodDeg);
        SmartDashboard.putNumber("Aim/DrumRPM", rpm);
        // Bearing to target relative to robot heading (0 = straight ahead).
        double hubAngleDeg = hub.minus(ShotMath.muzzle(pose.getTranslation(), pose.getRotation()))
            .getAngle().minus(pose.getRotation()).getDegrees();
        SmartDashboard.putNumber("Aim/HubAngleDeg", hubAngleDeg);

        m_hood.setHoodDegrees(hoodDeg);
    }

    @Override
    public void end(boolean interrupted) {
        // Release: stop driving so the stow-on-release binding takes over.
        m_hood.setHoodPercentage(0.0);
    }
}
