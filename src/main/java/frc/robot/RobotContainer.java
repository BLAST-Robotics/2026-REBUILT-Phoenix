package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.commands.AimAndShoot;
import frc.robot.commands.AimHood;
import frc.robot.commands.ShootOnTheMove;
import frc.robot.generated.TunerConstants;
import frc.robot.logging.FieldZones;
import frc.robot.logging.PDHLogger;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Shooter;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
    private double SpeedMultiplier = 1.0;

    private SlewRateLimiter xLimiter = new SlewRateLimiter(3.0);
    private SlewRateLimiter yLimiter = new SlewRateLimiter(3.0);
    private SlewRateLimiter rotationalLimiter = new SlewRateLimiter(3.0);

    /* Swerve drive */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    // Controllers: driver on port 0, operator on port 1.
    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);

    // Control set selector (Elastic dropdown): Match vs Test bench controls.
    private final SendableChooser<String> controlMode = new SendableChooser<>();
    private final Trigger testControls = new Trigger(() -> "Test".equals(controlMode.getSelected()));
    private final Trigger matchControls = testControls.negate();

    // Test-mode setpoints (signs match the match bindings).
    private static final double kTestShooterHighRpm = -6000.0;
    private static final double kTestShooterLowRpm = -2000.0;
    private static final double kTestSpindleRps = -40.0;

    // Competition shot: drum RPM + spindle + feed, all at once.
    private static final double kShootRpm = -6000.0;
    private static final double kShootSpindleRps = -40.0;

    // Intake rollers run at 0.4 duty everywhere.
    private static final double kIntakeRollerDuty = 0.4;

    // B-button deploy/stow toggle state.
    private boolean intakeDeployed = false;

    /** 0.3x slow-mode drive request, shared by match hold and test hold. */
    private SwerveRequest slowDriveRequest() {
        return drive.withVelocityX(xLimiter.calculate(-driver.getLeftY()) * MaxSpeed * SpeedMultiplier * 0.3)
            .withVelocityY(yLimiter.calculate(-driver.getLeftX()) * MaxSpeed * SpeedMultiplier * 0.3)
            .withRotationalRate(rotationalLimiter.calculate(-driver.getRightX()) * MaxAngularRate * SpeedMultiplier * 0.3);
    }

    /**
     * Aim-mode drive: X-brake lock while the driver is off the sticks, normal
     * field-centric drive while they move (shoot on the move), re-locking
     * when the sticks return to neutral.
     */
    private SwerveRequest aimDriveRequest() {
        double stickActivity = Math.abs(driver.getLeftY())
            + Math.abs(driver.getLeftX())
            + Math.abs(driver.getRightX());
        if (stickActivity > 0.2) {
            return drive.withVelocityX(xLimiter.calculate(-driver.getLeftY()) * MaxSpeed * SpeedMultiplier)
                .withVelocityY(yLimiter.calculate(-driver.getLeftX()) * MaxSpeed * SpeedMultiplier)
                .withRotationalRate(rotationalLimiter.calculate(-driver.getRightX()) * MaxAngularRate * SpeedMultiplier);
        }
        return brake;
    }

    // Subsystems
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final Intake intake = new Intake();
    private final Hopper hopper = new Hopper();
    private final Shooter shooter = new Shooter();
    private final Hood hood = new Hood();
    private final LED led = new LED();
    private final LimelightVision vision = new LimelightVision(drivetrain);
    private final PDHLogger pdh = new PDHLogger(16); // PDH 2.0 CAN id (todo: confirm)
    private final FieldZones fieldZones = new FieldZones(() -> drivetrain.getState().Pose);

    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        configureBindings();
        configureVisionLogging();

        MechanismTriggers.register(intake, hopper, shooter, hood);

        // Auto chooser from every auto on the robot (deploy/pathplanner/autos).
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        if (Constants.kTestControlsDefaultOnBoot) {
            controlMode.setDefaultOption("Test", "Test");
            controlMode.addOption("Match", "Match");
        } else {
            controlMode.setDefaultOption("Match", "Match");
            controlMode.addOption("Test", "Test");
        }
        SmartDashboard.putData("Control Mode", controlMode);

        pdh.populateDashboard();
        MechanismTriggers.logNames();
    }

    private void configureBindings() {
        /* ============ Drive ============ */
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(xLimiter.calculate(-driver.getLeftY()) * MaxSpeed * SpeedMultiplier)
                    .withVelocityY(yLimiter.calculate(-driver.getLeftX()) * MaxSpeed * SpeedMultiplier)
                    .withRotationalRate(rotationalLimiter.calculate(-driver.getRightX()) * MaxAngularRate * SpeedMultiplier)
            )
        );

        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        driver.rightBumper().and(matchControls).whileTrue(drivetrain.applyRequest(this::slowDriveRequest));
        driver.rightBumper().and(testControls).whileTrue(drivetrain.applyRequest(this::slowDriveRequest));

        driver.a().whileTrue(drivetrain.applyRequest(() -> brake));
        driver.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))
        ));
        driver.leftBumper().and(matchControls).onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
        driver.leftBumper().and(testControls).onTrue(drivetrain.runOnce(() -> drivetrain.getPigeon2().setYaw(0)));

        /* ============ Mechanisms (operator, competition) ============ */
        // RT: shoot (rev drums + spindle + feed).
        operator.rightTrigger().and(matchControls).whileTrue(
            shooter.revToRpm(kShootRpm)
                .alongWith(shooter.runSpindle(kShootSpindleRps))
                .alongWith(hopper.runForward()));
        // LT: intake rollers in.
        operator.leftTrigger().and(matchControls).whileTrue(intake.runRoller(kIntakeRollerDuty));
        // B: alternate deploy / stow on each press.
        operator.b().and(matchControls).onTrue(Commands.runOnce(() -> intakeDeployed = !intakeDeployed)
            .andThen(Commands.either(intake.deploy(), intake.store(), () -> intakeDeployed)));
        // RB toggle: aim (hood + drums, NO firing) with X-brake lock.
        // Move the driver sticks to unlock and drive (shoot on the move);
        // releasing them re-locks. Toggle off to fully release.
        operator.rightBumper().and(matchControls).toggleOnTrue(
            drivetrain.applyRequest(this::aimDriveRequest)
                .alongWith(new AimHood(drivetrain, hood, shooter)));
        // X: brake.
        operator.x().and(matchControls).whileTrue(drivetrain.applyRequest(() -> brake));

        // Aiming (uses Limelight pose -> interpolating tables).
        //operator.povUp().whileTrue(new AimAndShoot(drivetrain, hood, shooter));

        // Shoot on the move: keeps aiming at the target from the live pose while
        // the robot is driving, firing through the spindle continuously. Hold to
        // keep re-aiming as you travel.
        //operator.start().whileTrue(new ShootOnTheMove(drivetrain, hood, shooter));

        /* ============ Test controls (Control Mode = Test) ============ */
        // Shooter high/low: rev drums AND run spindle together.
        operator.povUp().and(testControls).whileTrue(
            shooter.revToRpm(kTestShooterHighRpm).alongWith(shooter.runSpindle(kTestSpindleRps)));
        operator.povDown().and(testControls).whileTrue(
            shooter.revToRpm(kTestShooterLowRpm).alongWith(shooter.runSpindle(kTestSpindleRps)));
        // Spindle standalone toggle.
        operator.povLeft().and(testControls).toggleOnTrue(shooter.runSpindle(kTestSpindleRps));
        // Stop everything on the shooter.
        operator.povRight().and(testControls).onTrue(shooter.stopAll());

        // Manual hood (left stick Y) and pivot (right stick Y). Flip the sign
        // if a direction runs backwards on your mechanism.
        testControls.and(() -> Math.abs(operator.getLeftY()) > 0.15).whileTrue(
            hood.runEnd(() -> hood.setHoodPercentage(-operator.getLeftY() * 0.3),
                () -> hood.setHoodPercentage(0.0)));
        testControls.and(() -> Math.abs(operator.getRightY()) > 0.15).whileTrue(
            intake.runEnd(() -> intake.setPivotPercentage(-operator.getRightY() * 0.3),
                () -> intake.setPivotPercentage(0.0)));

        // Intake deploy / stow.
        operator.x().and(testControls).whileTrue(intake.deploy());
        operator.y().and(testControls).whileTrue(intake.store());

        // Hood to ends: B = up (max), A = down/stowed (min).
        operator.b().and(testControls).whileTrue(hood.setAngle(Constants.Hood.kHoodMaxDegrees));
        operator.a().and(testControls).whileTrue(hood.setAngle(Constants.Hood.kHoodMinDegrees));

        // Intake roller + agitator.
        operator.rightBumper().and(testControls).whileTrue(intake.runRoller(kIntakeRollerDuty));
        operator.rightTrigger().and(testControls).whileTrue(hopper.runForward());

        /* ============ LEDs ============ */
        RobotModeTriggers.disabled().whileTrue(led.disabledStrobe());
        RobotModeTriggers.teleop().whileTrue(led.solidGold());
        RobotModeTriggers.autonomous().whileTrue(led.solidGreen());
    }

    /** Sends PathPlanner trajectory/robot pose to the dashboard (Elastic). */
    private void configureVisionLogging() {
        Field2d field = new Field2d();
        SmartDashboard.putData("Field", field);

        PathPlannerLogging.setLogActivePathCallback(
            path -> {
                var obj = field.getObject("traj");
                obj.setPoses(path);
            });
        PathPlannerLogging.setLogTargetPoseCallback(
            target -> field.getObject("target").setPose(target));
        PathPlannerLogging.setLogCurrentPoseCallback(field::setRobotPose);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}