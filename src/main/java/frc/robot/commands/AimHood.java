package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.AimTables;
import frc.robot.Constants;
import frc.robot.Constants.Aim;
import frc.robot.FieldConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;

/**
 * Aim only (no firing). Looks up hood angle and drum RPM for the distance to
 * the alliance hub and holds them. The spindle is never run here; fire with
 * the shoot button. Hood output is clamped into the measured soft-limit
 * window so a stale table can never slam the mechanism past its stops.
 */
public class AimHood extends Command {
    private final CommandSwerveDrivetrain m_drivetrain;
    private final Hood m_hood;
    private final Shooter m_shooter;
    private final AimTables m_aimTables;

    public AimHood(
        CommandSwerveDrivetrain drivetrain,
        Hood hood,
        Shooter shooter
    ) {
        m_drivetrain = drivetrain;
        m_hood = hood;
        m_shooter = shooter;
        m_aimTables = AimTables.getInstance();
        addRequirements(hood, shooter);
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

        // Aim at the hub, shifted back by how far the robot moves during the flight.
        Translation2d hub = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
            ? FieldConstants.kHubRed
            : FieldConstants.kHubBlue;

        double tof = 0.0;
        double distance = 0.0;
        double hoodDeg = 0.0;
        double rpm = 0.0;
        for (int i = 0; i < 3; i++) {
            distance = muzzle.getDistance(hub.minus(fieldVel.times(tof)));
            hoodDeg = m_aimTables.hoodDegreesForDistance(distance);
            rpm = m_aimTables.shooterRpmForDistance(distance);
            double exitSpeed = Aim.kExitPerDrumRps * (rpm / 60.0);
            double horizSpeed = exitSpeed * Math.cos(Math.toRadians(hoodDeg));
            tof = horizSpeed > 1e-6 ? distance / horizSpeed : 0.0;
        }

        // Clamp into the measured mechanism window (tune tables to this frame).
        hoodDeg = Math.max(Constants.Hood.kHoodMinDegrees,
            Math.min(Constants.Hood.kHoodMaxDegrees, hoodDeg));

        SmartDashboard.putNumber("Aim/Distance", distance);
        SmartDashboard.putNumber("Aim/HoodDeg", hoodDeg);
        SmartDashboard.putNumber("Aim/DrumRPM", rpm);

        m_hood.setHoodDegrees(hoodDeg);
        // Tables are positive; the tested drum direction is negative.
        m_shooter.setDrumRpm(-rpm);
    }

    @Override
    public void end(boolean interrupted) {
        m_shooter.setDrumPercentage(0.0);
    }
}
