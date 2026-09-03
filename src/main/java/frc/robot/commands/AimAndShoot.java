package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.AimTables;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;

/**
 * Aim and shoot. Reads the robot's field pose (fusion of odom + Limelight
 * MegaTag2), computes the distance to the target, looks up the hood angle and
 * shooter RPM from the {@link InterpolatingDoubleTreeMap}s, then drives the
 * hood and shooter to those setpoints and runs the spindle to fire.
 *
 * <p>The Limelight is not used for direct aim - it just improves the field pose
 * used to compute distance. Once the pose is accurate, distance does the rest.
 */
public class AimAndShoot extends Command {
    private final CommandSwerveDrivetrain m_drivetrain;
    private final Hood m_hood;
    private final Shooter m_shooter;
    private final AimTables m_aimTables;

    // Target is the center of the scoring target. Placeholder for the 2026 field;
    // replace with the actual target pose once known.
    private static final double kTargetX = 16.54; // meters
    private static final double kTargetY = 8.02;  // meters

    public AimAndShoot(
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
        double distance = m_drivetrain.getState().Pose
            .getTranslation()
            .getDistance(new Translation2d(kTargetX, kTargetY));

        double hoodDeg = m_aimTables.hoodDegreesForDistance(distance);
        double rpm = m_aimTables.shooterRpmForDistance(distance);

        SmartDashboard.putNumber("Aim/Distance", distance);
        SmartDashboard.putNumber("Aim/HoodDeg", hoodDeg);
        SmartDashboard.putNumber("Aim/DrumRPM", rpm);

        m_hood.setHoodDegrees(hoodDeg);
        m_shooter.setDrumRpm(rpm);
        m_shooter.setSpindleRps(80.0);
    }

    @Override
    public void end(boolean interrupted) {
        m_shooter.stopAll();
    }
}