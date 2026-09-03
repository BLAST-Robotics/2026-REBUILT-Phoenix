package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.AimTables;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;

/**
 * Shoot-on-the-move. Continuously recomputes the hood angle and shooter RPM
 * from the robot's live field pose (odom + Limelight MegaTag2) while the driver
 * keeps the robot moving, and fires. Unlike {@link AimAndShoot} this never
 * finishes on its own - it keeps re-aiming as the robot travels so the shot is
 * correct at the moment of release.
 *
 * <p>The drive is deliberately left to the driver separate from this command.
 */
public class ShootOnTheMove extends Command {
    private final CommandSwerveDrivetrain m_drivetrain;
    private final Hood m_hood;
    private final Shooter m_shooter;
    private final AimTables m_aimTables;

    // Shot + target origin offsets (meters). Refine once the shot origin and
    // target height/offset are known for the 2026 field.
    private static final double kTargetX = 16.54; // meters
    private static final double kTargetY = 8.02;  // meters
    private static final double kSpindleRps = 80.0; // roller spindle setpoint

    public ShootOnTheMove(
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
    public void initialize() {
        // Nothing to do at start; loop keeps us aimed.
    }

    @Override
    public void execute() {
        // Distance to the target from the live pose.
        double distance = m_drivetrain.getState().Pose
            .getTranslation()
            .getDistance(new Translation2d(kTargetX, kTargetY));

        double hoodDeg = m_aimTables.hoodDegreesForDistance(distance);
        double rpm = m_aimTables.shooterRpmForDistance(distance);

        SmartDashboard.putNumber("Aim/Distance", distance);
        SmartDashboard.putNumber("Aim/HoodDeg", hoodDeg);
        SmartDashboard.putNumber("Aim/DrumRPM", rpm);

        // Keep re-aiming every loop while moving.
        m_hood.setHoodDegrees(hoodDeg);
        m_shooter.setDrumRpm(rpm);
        m_shooter.setSpindleRps(kSpindleRps);
    }

    @Override
    public void end(boolean interrupted) {
        m_shooter.stopAll();
    }
}