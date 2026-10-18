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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;

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

    // Control set selector (Elastic toggle switch): Match vs Test bench controls.
    private final Trigger testControls =
        new Trigger(() -> SmartDashboard.getBoolean("Test Mode", Constants.kTestControlsDefaultOnBoot));
    private final Trigger matchControls = testControls.negate();

    // Test-mode setpoints (signs match the match bindings).
    private static final double kTestShooterHighRpm = -3300.0;
    private static final double kTestShooterLowRpm = -2000.0;
    private static final double kTestSpindleRps = -50.0;

    // Competition shot (under the 3600 RPM drum ceiling so gates open).
    private static final double kShootRpm = -3300.0;
    private static final double kShootSpindleRps = -50.0;

    // Intake roller base (RPS now, closed-loop; was 0.4 duty).

    // RB aim-toggle state (explicit: onFalse stow would cancel a
    // toggleOnTrue latch on every release, so the toggle owns its flag).
    private boolean m_aimToggled = false;

    // Intake deploy/stow toggle state, shared by both pads.
    private boolean m_intakeDeployed = false;

    // Aim-toggle shoot-on-the-move: last time the driver translated.
    private double m_lastTranslateMoveS = 0.0;
    private static final double kAimLockDelayS = 0.5;
    private static final double kAimHeadingKp = 4.0;

    /** 0.3x slow-mode drive request, shared by match hold and test hold. */
    private SwerveRequest slowDriveRequest() {
        return drive.withVelocityX(xLimiter.calculate(-driver.getLeftY()) * MaxSpeed * SpeedMultiplier * 0.3)
            .withVelocityY(yLimiter.calculate(-driver.getLeftX()) * MaxSpeed * SpeedMultiplier * 0.3)
            .withRotationalRate(rotationalLimiter.calculate(-driver.getRightX()) * MaxAngularRate * SpeedMultiplier * 0.3);
    }

    /**
     * Aim-toggle drive: BACK locked on the target (shooter side) always;
     * translation follows the driver sticks with NO slew limiting so
     * shoot-on-the-move answers instantly. X-brake re-engages 500 ms after
     * the sticks go still.
     *
     * <p>The target is the hub while the robot is inside its own alliance
     * zone, and switches automatically to a hub-clearing point in that zone
     * (a pass) once the robot leaves it.
     */
    private SwerveRequest aimDriveRequest() {
        boolean passing = passing();
        SmartDashboard.putBoolean("Aim/Active", true);
        SmartDashboard.putBoolean("Aim/Passing", passing);
        SmartDashboard.putBoolean("Aim/AlignedToHub", hood.hoodAtTarget());
        var pose = drivetrain.getState().Pose;
        Translation2d target = currentAimPoint();
        // Back of the robot faces the target: desired heading = bearing + 180.
        double errRad = target.minus(pose.getTranslation()).getAngle()
            .plus(Rotation2d.fromDegrees(180.0))
            .minus(pose.getRotation()).getRadians();
        double lockRate = Math.max(-MaxAngularRate * 0.6,
            Math.min(MaxAngularRate * 0.6, kAimHeadingKp * errRad));
        if (Math.abs(errRad) < Math.toRadians(0.5)) {
            lockRate = 0.0;
        }

        double translateActivity = Math.abs(driver.getLeftY()) + Math.abs(driver.getLeftX());
        if (translateActivity > 0.15) {
            m_lastTranslateMoveS = Timer.getFPGATimestamp();
            // Slow mode (RB, either pad) scales translation only: the hub
            // heading lock and hood tracking keep running, so slowing down
            // never cancels shoot-on-the-move.
            double slow = (driver.rightBumper().getAsBoolean()
                || operator.rightBumper().getAsBoolean()) ? 0.3 : 1.0;
            SmartDashboard.putBoolean("Aim/Slow", slow < 1.0);
            return drive.withVelocityX(-driver.getLeftY() * MaxSpeed * SpeedMultiplier * slow)
                .withVelocityY(-driver.getLeftX() * MaxSpeed * SpeedMultiplier * slow)
                .withRotationalRate(lockRate);
        }
        // Still: keep turning toward the hub until faced (2 deg), then
        // simply stop — straight modules hold fine, no X-brake needed.
        boolean faced = Math.abs(errRad) < Math.toRadians(2.0);
        boolean stillLong = Timer.getFPGATimestamp() - m_lastTranslateMoveS > kAimLockDelayS;
        if (stillLong && faced) {
            return drive.withVelocityX(0.0).withVelocityY(0.0).withRotationalRate(0.0);
        }
        return drive.withVelocityX(0.0).withVelocityY(0.0).withRotationalRate(lockRate);
    }

    /**
     * Passing engages automatically: while auto-aim is toggled on, the muzzle
     * aims into the own alliance zone once the robot has left that zone.
     * Inside the zone the hub is the target, so both are handled by
     * {@link #currentAimPoint()}.
     */
    private boolean passing() {
        if (!m_aimToggled) {
            return false;
        }
        boolean red = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
            == DriverStation.Alliance.Red;
        String ownZone = red ? FieldZones.kRedZone : FieldZones.kBlueZone;
        return !ownZone.equals(FieldZones.zoneFor(drivetrain.getState().Pose.getX()));
    }

    /** Aim point for the muzzle: the hub, or a hub-clearing pass point in the own zone. */
    private Translation2d currentAimPoint() {
        boolean red = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
            == DriverStation.Alliance.Red;
        return passing()
            ? PassGeometry.safeAimPoint(drivetrain.getState().Pose.getTranslation(), red)
            : PassGeometry.hub(red);
    }

    // Subsystems
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final Intake intake = new Intake();
    private final Hopper hopper = new Hopper();
    private final Shooter shooter = new Shooter();
    private final Hood hood = new Hood();
    private final LED led = new LED();
    private final LimelightVision vision = new LimelightVision(drivetrain, logger.field);
    private final PDHLogger pdh = new PDHLogger(16); // PDH 2.0 CAN id (todo: confirm)
    private final FieldZones fieldZones = new FieldZones(() -> drivetrain.getState().Pose);

    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        // Stream pose + module state to NT (Elastic field widgets read these).
        drivetrain.registerTelemetry(logger::telemeterize);
        configureBindings();
        configureVisionLogging();

        MechanismTriggers.register(intake, hopper, shooter, hood);

        // Auto chooser from every auto on the robot (deploy/pathplanner/autos).
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        SmartDashboard.putBoolean("Test Mode", Constants.kTestControlsDefaultOnBoot);

        pdh.populateDashboard();
        MechanismTriggers.logNames();

        reduceSwerveCanLoad();
    }

    /**
     * The rio bus carries 24 devices and was dropping frames, starving
     * mechanism feedback. Odometry-critical signals (position/velocity/
     * absolute) are left exactly as CTRE configured them; everything
     * auxiliary on the 8 swerve Talons + 4 turn encoders goes slow.
     */
    private void reduceSwerveCanLoad() {
        for (var module : drivetrain.getModules()) {
            var drive = module.getDriveMotor();
            var steer = module.getSteerMotor();
            for (var talon : new com.ctre.phoenix6.hardware.TalonFX[]{drive, steer}) {
                talon.getDeviceTemp().setUpdateFrequency(4);
                talon.getStatorCurrent().setUpdateFrequency(10);
                talon.getSupplyCurrent().setUpdateFrequency(10);
                talon.getMotorVoltage().setUpdateFrequency(10);
                talon.getClosedLoopError().setUpdateFrequency(10);
                talon.getClosedLoopOutput().setUpdateFrequency(10);
                talon.getFault_Hardware().setUpdateFrequency(4);
            }
        }
    }

    private void configureBindings() {
        /* ============ Neutral modes (subsystems, not swerve) ============ */
        // Disabled: everything coasts (safe to push, mechanisms settle).
        // Held for the whole disabled period, not just the transition: a
        // one-shot onTrue can't undo a MotionMagic request issued before the
        // transition (closed-loop requests persist in hardware otherwise and
        // "come back to life"), and these commands preempt anything still
        // running by taking their subsystems' requirements. Pivots to coast,
        // spin/roller motors to zero output (coast neutral mode); feed latch out.
        RobotModeTriggers.disabled().whileTrue(
            intake.coastHold()
                .alongWith(hood.coastHold(), shooter.coastHold(), hopper.coastHold()));
        // Enabled (auto + teleop): pivots brake so gravity can't sag the
        // boot-relative datum; spin/roller motors stay coast always.
        RobotModeTriggers.autonomous().onTrue(Commands.runOnce(() -> {
            hood.setBrakeMode(true);
            intake.setPivotBrakeMode(true);
        }, hood, intake));
        RobotModeTriggers.teleop().onTrue(Commands.runOnce(() -> {
            hood.setBrakeMode(true);
            intake.setPivotBrakeMode(true);
        }, hood, intake));

        /* ============ Drive ============ */
        // Aim-toggle state (LT latch): defined up here because the slow-mode
        // bindings below need it — slow mode yields to aim, never fights it.
        Trigger aimActive = new Trigger(() -> m_aimToggled).and(matchControls);
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> {
                SmartDashboard.putBoolean("Aim/Active", false);
                SmartDashboard.putBoolean("Aim/AlignedToHub", false);
                return drive.withVelocityX(xLimiter.calculate(-driver.getLeftY()) * MaxSpeed * SpeedMultiplier)
                    .withVelocityY(yLimiter.calculate(-driver.getLeftX()) * MaxSpeed * SpeedMultiplier)
                    .withRotationalRate(rotationalLimiter.calculate(-driver.getRightX()) * MaxAngularRate * SpeedMultiplier);
            })
        );

        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // Match slow mode yields while aim is toggled: the aim request owns
        // the drivetrain there and applies the 0.3x itself (translation only),
        // so holding RB slows down without breaking the hub lock or hood track.
        driver.rightBumper().and(matchControls).and(aimActive.negate()).whileTrue(drivetrain.applyRequest(this::slowDriveRequest));
        driver.rightBumper().and(testControls).whileTrue(drivetrain.applyRequest(this::slowDriveRequest));

        driver.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // (B point-wheels retired in Match: B is intake deploy. Test keeps it.)
        driver.b().and(testControls).whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))
        ));
        driver.leftBumper().and(matchControls).onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
        driver.leftBumper().and(testControls).onTrue(drivetrain.runOnce(() -> drivetrain.getPigeon2().setYaw(0)));

        /* ============ Mechanisms (driver + operator copy, competition) ============ */
        // Every mechanism binding below exists on BOTH pads (sticks stay
        // driver-only). Shared command instances: pressing both pads runs it
        // once; releasing one pad while the other holds causes a one-loop
        // reschedule blip. Coordinate handoffs.
        var shootSequence = shooter.shootAtRpm(kShootRpm, kShootSpindleRps,
                Constants.Shooter.kSpindleIdleRPS, hood)
            .alongWith(hopper.feedWhenReady(shooter,
                Constants.Hopper.kAgitatorIdleRPS, Constants.Hopper.kAgitatorMaxRPS));
        // Match idle: spindle trickle + agitator trickle whenever enabled.
        // RT shooting overrides both while held (superseded commands swap).
        Trigger neitherTrigger = driver.rightTrigger().negate()
            .and(operator.rightTrigger().negate()).and(matchControls)
            .and(RobotModeTriggers.disabled().negate());
        neitherTrigger.whileTrue(shooter.runSpindle(Constants.Shooter.kSpindleIdleRPS));
        neitherTrigger.whileTrue(
            hopper.runEnd(() -> hopper.setAgitatorRps(Constants.Hopper.kAgitatorIdleRPS),
                () -> hopper.setAgitatorPercentage(0.0)));
        // Enable-time kick: whileTrue above only fires on transitions, so a
        // Match-already-selected enable would never start the trickle until
        // the chooser wiggled. Separate instances; RT shooting supersedes.
        var enableSpindleIdle = shooter.runSpindle(Constants.Shooter.kSpindleIdleRPS);
        var enableAgitIdle = hopper.runEnd(
            () -> hopper.setAgitatorRps(Constants.Hopper.kAgitatorIdleRPS),
            () -> hopper.setAgitatorPercentage(0.0));
        RobotModeTriggers.teleop().and(matchControls).onTrue(enableSpindleIdle);
        RobotModeTriggers.teleop().and(matchControls).onTrue(enableAgitIdle);
        RobotModeTriggers.autonomous().and(matchControls).onTrue(enableSpindleIdle);
        RobotModeTriggers.autonomous().and(matchControls).onTrue(enableAgitIdle);
        // RT: shoot. Drums rev; spindle + agitator stay at trickle until the
        // drums are up to speed, then kick to full feed. Intake dips up over
        // 4 s on press, drops back to deployed at the end.
        driver.rightTrigger().and(matchControls).whileTrue(shootSequence);
        operator.rightTrigger().and(matchControls).whileTrue(shootSequence);
        driver.rightTrigger().and(matchControls).onTrue(
            Commands.runOnce(() -> shooter.resetFeedLatch()).andThen(intake.shootDip()));
        operator.rightTrigger().and(matchControls).onTrue(
            Commands.runOnce(() -> shooter.resetFeedLatch()).andThen(intake.shootDip()));
        // B: deploy/stow toggle (both pads flip the same latch).
        // The toggle ONLY flips state + sets the pivot target (both instant,
        // always-ending); rollers run off the latch trigger below. Earlier
        // revisions chained never-ending roller phases into the press, which
        // deadlocked every press after the first two (same-instance
        // reschedule is ignored while running).
        driver.b().and(matchControls).onTrue(Commands.runOnce(() -> {
            m_intakeDeployed = !m_intakeDeployed;
            intake.setPivotTarget(m_intakeDeployed
                ? Constants.Intake.kPivotDeployedDelta
                : Constants.Intake.kPivotStowedDelta);
        }));
        operator.b().and(matchControls).onTrue(Commands.runOnce(() -> {
            m_intakeDeployed = !m_intakeDeployed;
            intake.setPivotTarget(m_intakeDeployed
                ? Constants.Intake.kPivotDeployedDelta
                : Constants.Intake.kPivotStowedDelta);
        }));
        // Rollers follow the latch (smart 40 RPS). Dip/test/auto commands
        // requiring intake supersede while they run, then this resumes.
        // Enabled-only: nothing auto-runs while disabled.
        new Trigger(() -> m_intakeDeployed).and(matchControls)
            .and(RobotModeTriggers.disabled().negate())
            .whileTrue(intake.runRoller(Constants.Intake.kRollerBaseRPS));
        // LT toggle: aim mode. Heading locked on the hub (hood tracks too,
        // shooter NEVER spins). Move sticks for shoot-on-the-move; 500 ms
        // still re-brakes. RB slows translation to 0.3x without breaking the
        // lock. Passing is automatic: outside the own alliance zone the aim
        // point switches to the own zone, dodging the hub if it is in the way.
        // Second press releases and stows the hood.
        // (A raw onFalse stow can't do this: it fires on every release and
        // would cancel the latched toggle — hence the explicit state.)
        driver.leftTrigger().and(matchControls)
            .onTrue(Commands.runOnce(() -> m_aimToggled = !m_aimToggled));
        operator.leftTrigger().and(matchControls)
            .onTrue(Commands.runOnce(() -> m_aimToggled = !m_aimToggled));
        aimActive.whileTrue(drivetrain.applyRequest(this::aimDriveRequest)
            .alongWith(new AimHood(drivetrain, hood, this::currentAimPoint)));
        // Stow on toggle-off only while enabled: firing this through a
        // disable would schedule a ghost stow that drives the hood on the
        // next enable (outputs are dead in disabled, the command isn't).
        aimActive.onFalse(Commands.either(hood.setAngle(0.0), Commands.none(),
            DriverStation::isEnabled));
        RobotModeTriggers.disabled()
            .onTrue(Commands.runOnce(() -> m_aimToggled = false));
        // X: reverse intake rollers (both pads, jam clearing).
        driver.x().and(matchControls).whileTrue(intake.runRoller(-0.5));
        operator.x().and(matchControls).whileTrue(intake.runRoller(-0.5));
        operator.a().and(matchControls).whileTrue(drivetrain.applyRequest(() -> brake));
        // Slow 0.3x (both pads) + seed (both pads, match).
        // Match slow yields to aim (see drive section): RB while aimed only
        // scales the aim request's translation.
        operator.rightBumper().and(matchControls).and(aimActive.negate())
            .whileTrue(drivetrain.applyRequest(this::slowDriveRequest));
        operator.leftBumper().and(matchControls)
            .onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        // Aiming (uses Limelight pose -> interpolating tables).
        //operator.povUp().whileTrue(new AimAndShoot(drivetrain, hood, shooter));

        // Shoot on the move: keeps aiming at the target from the live pose while
        // the robot is driving, firing through the spindle continuously. Hold to
        // keep re-aiming as you travel.
        //operator.start().whileTrue(new ShootOnTheMove(drivetrain, hood, shooter));

        /* ============ Test controls (Control Mode = Test) ============ */
        // Shooter high/low: rev drums + spindle together.
        operator.povUp().and(testControls).whileTrue(
            shooter.revAndFeed(kTestShooterHighRpm, kTestSpindleRps));
        operator.povDown().and(testControls).whileTrue(
            shooter.revAndFeed(kTestShooterLowRpm, kTestSpindleRps));
        // Spindle standalone toggle.
        operator.povLeft().and(testControls).toggleOnTrue(shooter.runSpindle(kTestSpindleRps));
        // Stop everything on the shooter.
        operator.povRight().and(testControls).onTrue(shooter.stopAll());

        // Manual hood: percent jog, direction owned by motor configs.
        // Manual yields to the preset buttons (same subsystem).
        testControls.and(() -> Math.abs(operator.getLeftY()) > 0.15)
            .and(operator.a().negate()).and(operator.b().negate()).whileTrue(
            hood.runEnd(() -> hood.setHoodPercentage(operator.getLeftY() * 0.3),
                () -> hood.setHoodPercentage(0.0)));
        testControls.and(() -> Math.abs(operator.getRightY()) > 0.15)
            .and(operator.x().negate()).and(operator.y().negate()).whileTrue(
            intake.runEnd(() -> intake.setPivotPercentage(-operator.getRightY() * 0.3),
                () -> intake.setPivotPercentage(0.0)));

        // Intake deploy / stow.
        operator.x().and(testControls).whileTrue(intake.deploy());
        operator.y().and(testControls).whileTrue(intake.store());

        // Hood to ends (boot-relative deltas): B = deployed, A = stowed.
        operator.b().and(testControls).whileTrue(hood.setAngle(Constants.Hood.kHoodTravelDegrees));
        operator.a().and(testControls).whileTrue(hood.setAngle(0.0));

        // Intake roller + agitator.
        operator.rightBumper().and(testControls).whileTrue(intake.runRoller(20.0));
        operator.rightTrigger().and(testControls).whileTrue(hopper.runForward());

        // Button/mode debug: edge-triggered, no subsystem requirements.
        SmartDashboard.putString("Debug/LastBtn", "none");
        SmartDashboard.putBoolean("Debug/TestMode", false);
        operator.a().onTrue(Commands.runOnce(() -> SmartDashboard.putString("Debug/LastBtn", "A")));
        operator.b().onTrue(Commands.runOnce(() -> SmartDashboard.putString("Debug/LastBtn", "B")));
        testControls.onTrue(Commands.runOnce(() -> SmartDashboard.putBoolean("Debug/TestMode", true)));
        testControls.onFalse(Commands.runOnce(() -> SmartDashboard.putBoolean("Debug/TestMode", false)));

        /* ============ LEDs ============ */
        RobotModeTriggers.disabled().whileTrue(led.disabledStrobe());
        RobotModeTriggers.teleop().whileTrue(led.solidGold());
        RobotModeTriggers.autonomous().whileTrue(led.solidGreen());
    }

    /** Sends PathPlanner trajectory/robot pose to the dashboard (Elastic). */
    private void configureVisionLogging() {
        // Shared with Telemetry: same /SmartDashboard/Field the widget reads.
        Field2d field = logger.field;

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