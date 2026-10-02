package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;

/**
 * Registers named commands that auto paths can trigger via event markers,
 * e.g. "{IntakeDeploy}" in PathPlanner maps to "IntakeDeploy" below.
 */
public final class MechanismTriggers {

    private MechanismTriggers() {}

    /** Call once at robot start after all subsystems are constructed. */
    public static void register(
        frc.robot.subsystems.Intake intake,
        frc.robot.subsystems.Hopper hopper,
        frc.robot.subsystems.Shooter shooter,
        frc.robot.subsystems.Hood hood
    ) {
        // Intake
        NamedCommands.registerCommand("IntakeDeploy", intake.deploy());
        NamedCommands.registerCommand("IntakeStore", intake.store());
        NamedCommands.registerCommand("IntakeRoller", intake.runRoller(1.0));
        NamedCommands.registerCommand("IntakeRollerReverse", intake.runRoller(-0.5));

        // Hopper / agitator
        NamedCommands.registerCommand("AgitatorForward", hopper.runForward());
        NamedCommands.registerCommand("AgitatorReverse", hopper.runReverse());
        NamedCommands.registerCommand("AgitatorStop", hopper.stop());

        // Shooter (spinning up)
        NamedCommands.registerCommand("ShooterRev", shooter.revToRpm(
            AimTables.getInstance().shooterRpmForDistance(3.0)));
        NamedCommands.registerCommand("ShooterStop", shooter.stopAll());

        // Keep the shooter revved and run the spindle to fire.
        NamedCommands.registerCommand("ShooterSpit", Commands.sequence(
            shooter.revToRpm(AimTables.getInstance().shooterRpmForDistance(3.0)),
            shooter.runSpindle(80.0)
        ));

        // Hood
        NamedCommands.registerCommand("HoodAngle", hood.setAngle(
            AimTables.getInstance().hoodDegreesForDistance(3.0)));
    }

    /** Publish the available trigger names for reference. */
    public static void logNames() {
        SmartDashboard.putStringArray("PathPlanner/Triggers", new String[] {
            "IntakeDeploy", "IntakeStore", "IntakeRoller", "IntakeRollerReverse",
            "AgitatorForward", "AgitatorReverse", "AgitatorStop",
            "ShooterRev", "ShooterStop", "ShooterSpit", "HoodAngle"
        });
    }
}