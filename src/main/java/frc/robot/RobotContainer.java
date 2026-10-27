package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.commands.AimHood;
import frc.robot.generated.TunerConstants;
import frc.robot.logging.FieldZones;
import frc.robot.logging.PDHLogger;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Shooter;
import frc.robot.util.ShotMath;

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
    // Aim lock: identical to drive but with NO rotational deadband. The hub
    // heading correction is a computed rate (not a stick), and the stick-drift
    // deadband was swallowing the small corrections, leaving the robot a few
    // degrees off target no matter how high the gain got.
    private final SwerveRequest.FieldCentric aimDrive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(0.0)
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

    // Test-mode setpoints. Drum RPM is always a POSITIVE magnitude (the
    // feeding direction is applied inside Shooter). Spindle keeps its own sign.
    private static final double kTestShooterHighRpm = 6000.0;
    private static final double kTestShooterLowRpm = 2000.0;
    private static final double kTestSpindleRps = -50.0;

    // Competition shot. Drum RPM is now live from the distance table (see
    // aimShootRpm), so it tracks range like the hood does instead of a fixed
    // setpoint.
    private static final double kShootSpindleRps = -30.0;

    // Drive translation/rotation scale while the shoot trigger is held: keeps
    // the swerve from starving the drum of voltage mid-shot.
    private static final double kShootDriveScale = 0.3;

    // Optional upward pad on the table RPM. 0 = shoot the raw table value.
    private static final double kShootRpmMargin = 0.0;

    // Intake roller base (RPS now, closed-loop; was 0.4 duty).

    // RB aim-toggle state (explicit: onFalse stow would cancel a
    // toggleOnTrue latch on every release, so the toggle owns its flag).

    // Intake deploy/stow toggle state, shared by both pads.
    private boolean m_intakeDeployed = false;
    // Outtake (X) is a HOLD while deployed. The intake button cancels the
    // current hold (resumes intake) instead of stowing; it only stows when the
    // intake is running normally.
    private boolean m_outtakeSuppressed = false;

    // One-shot boot-homing latch (see Intake.homeToNearestEnd).
    private boolean m_intakeHomed = false;

    // Auto shoot hold. ShootStart schedules these in the background and
    // ShootEnd cancels them, so a PathPlanner Wait can sit between the two.
    private Command m_autoShootHold;
    private Command m_autoShootDip;

    // POV manual tuning (match controls, both used only while tuning).
    // Direct mechanism control, deliberately NOT connected to the aim table
    // or the shoot sequence.
    private double m_manualShooterRpm = 0.0;
    private double m_manualHoodDeg = 0.0;

    private static final double kAimHeadingKp = 8.0;
    // Aligned-to-hub heading tolerance (deg) and aim rotation-rate cap as a
    // fraction of MaxAngularRate. Tighter tolerance + higher gains = snappier.
    private static final double kAimHeadingTolDeg = 1.0;
    private static final double kAimHeadingRateFrac = 1.0;

    // While aimed, the driver may nudge the aim heading with the right stick at
    // this fraction of MaxAngularRate. The nudge accumulates into a heading
    // offset that holds while aiming and is cleared when aim is released, so it
    // never leaks into normal driving.
    private static final double kAimManualRotFrac = 0.2;
    private static final double kAimHeadingOffsetMaxDeg = 20.0;
    private double m_aimHeadingOffsetRad = 0.0;
    private double m_aimOffsetLastTime = 0.0;

    /** 0.3x slow-mode drive request, shared by match hold and test hold. */
    private SwerveRequest slowDriveRequest() {
        return drive.withVelocityX(xLimiter.calculate(-driver.getLeftY()) * MaxSpeed * SpeedMultiplier * 0.3)
            .withVelocityY(yLimiter.calculate(-driver.getLeftX()) * MaxSpeed * SpeedMultiplier * 0.3)
            .withRotationalRate(rotationalLimiter.calculate(-driver.getRightX()) * MaxAngularRate * SpeedMultiplier * 0.3);
    }

    /**
     * Aim-toggle drive: BACK locked on the target (shooter side) always;
     * translation follows the driver sticks with NO slew limiting so
     * shoot-on-the-move answers instantly. When the sticks go still the
     * rotation stops as soon as the robot is faced (no settle delay).
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
        // Driver heading nudge: accumulate the right stick at 0.2x rate into a
        // held offset (clamped), so the driver can rotate slightly while aimed.
        double now = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
        double dt = m_aimOffsetLastTime <= 0.0 ? 0.0
            : Math.min(0.1, now - m_aimOffsetLastTime);
        m_aimOffsetLastTime = now;
        m_aimHeadingOffsetRad += -driver.getRightX() * MaxAngularRate
            * kAimManualRotFrac * dt;
        double maxOffset = Math.toRadians(kAimHeadingOffsetMaxDeg);
        m_aimHeadingOffsetRad = Math.max(-maxOffset,
            Math.min(maxOffset, m_aimHeadingOffsetRad));
        SmartDashboard.putNumber("Aim/HeadingOffsetDeg",
            Math.toDegrees(m_aimHeadingOffsetRad));
        // Back of the robot faces the target: desired heading = bearing + 180,
        // plus any driver nudge.
        double errRad = target.minus(pose.getTranslation()).getAngle()
            .plus(Rotation2d.fromDegrees(180.0))
            .plus(Rotation2d.fromRadians(m_aimHeadingOffsetRad))
            .minus(pose.getRotation()).getRadians();
        double lockRate = Math.max(-MaxAngularRate * kAimHeadingRateFrac,
            Math.min(MaxAngularRate * kAimHeadingRateFrac, kAimHeadingKp * errRad));
        if (Math.abs(errRad) < Math.toRadians(kAimHeadingTolDeg * 0.5)) {
            lockRate = 0.0;
        }

        double translateActivity = Math.abs(driver.getLeftY()) + Math.abs(driver.getLeftX());
        if (translateActivity > 0.15) {
            // Slow mode scales translation only: the hub heading lock and hood
            // tracking keep running, so slowing down never cancels
            // shoot-on-the-move. Triggers on RB (either pad) OR any shoot
            // trigger, so shooting auto-slows to protect the drum.
            double slow = (driver.rightBumper().getAsBoolean()
                || operator.rightBumper().getAsBoolean()
                || shootingNow()) ? kShootDriveScale : 1.0;
            SmartDashboard.putBoolean("Aim/Slow", slow < 1.0);
            SmartDashboard.putBoolean("Aim/ShootSlow", shootingNow());
            return aimDrive.withVelocityX(-driver.getLeftY() * MaxSpeed * SpeedMultiplier * slow)
                .withVelocityY(-driver.getLeftX() * MaxSpeed * SpeedMultiplier * slow)
                .withRotationalRate(lockRate);
        }
        // Still: keep turning toward the target until faced
        // (kAimHeadingTolDeg), then simply stop — straight modules hold fine,
        // no X-brake needed.
        boolean faced = Math.abs(errRad) < Math.toRadians(kAimHeadingTolDeg);
        if (faced) {
            return aimDrive.withVelocityX(0.0).withVelocityY(0.0).withRotationalRate(0.0);
        }
        return aimDrive.withVelocityX(0.0).withVelocityY(0.0).withRotationalRate(lockRate);
    }

    /** Drops the driver's aim-heading nudge (called when aim is released). */
    private void resetAimHeadingOffset() {
        m_aimHeadingOffsetRad = 0.0;
        m_aimOffsetLastTime = 0.0;
        SmartDashboard.putNumber("Aim/HeadingOffsetDeg", 0.0);
    }

    /**
     * Passing engages automatically: while the shoot hold is active, the
     * muzzle aims into the own alliance zone once the robot has left that
     * zone. Inside the zone the hub is the target, so both are handled by
     * {@link #currentAimPoint()}.
     */
    private boolean passing() {
        if (!shootingNow()) {
            return false;
        }
        boolean red = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
            == DriverStation.Alliance.Red;
        String ownZone = red ? FieldZones.kRedZone : FieldZones.kBlueZone;
        return !ownZone.equals(FieldZones.zoneFor(drivetrain.getState().Pose.getX()));
    }

    /**
     * The full aim-and-shoot hold recipe: hub heading lock, hood tracking,
     * drum rev + spindle + agitator feed. Built fresh on every call because a
     * composed WPILib Command may not be added to a second composition — both
     * pads share the recipe, never the instance.
     */
    private Command aimAndShoot() {
        return drivetrain.applyRequest(this::aimDriveRequest)
            .alongWith(
                new AimHood(drivetrain, hood, this::currentAimPoint),
                shooter.shootAtRpm(this::aimShootRpm, kShootSpindleRps,
                    Constants.Shooter.kSpindleIdleRPS)
                    .alongWith(hopper.feedWhenReady(shooter,
                        Constants.Hopper.kAgitatorIdleRPS,
                        Constants.Hopper.kAgitatorMaxRPS)));
    }

    /** True while either pad holds the shoot trigger in Match control. */
    private boolean shootingNow() {
        return matchControls.getAsBoolean()
            && (driver.rightTrigger().getAsBoolean() || operator.rightTrigger().getAsBoolean());
    }

    /** Aim point for the muzzle: the hub, or a hub-clearing pass point in the own zone. */
    private Translation2d currentAimPoint() {
        boolean red = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
            == DriverStation.Alliance.Red;
        return passing()
            ? PassGeometry.safeAimPoint(drivetrain.getState().Pose.getTranslation(), red)
            : PassGeometry.hub(red);
    }

    /**
     * Live drum RPM for the current aim point from the distance table, using
     * the same flight-time lead as AimHood. Returned as a POSITIVE magnitude
     * (the feeding direction is applied inside Shooter).
     */
    private double aimShootRpm() {
        var pose = drivetrain.getState().Pose;
        var speeds = drivetrain.getState().Speeds;
        Translation2d fieldVel = new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond)
            .rotateBy(pose.getRotation());
        var shot = ShotMath.solve(
            ShotMath.muzzle(pose.getTranslation(), pose.getRotation()),
            fieldVel, currentAimPoint());
        double commanded = shot.rpm() + kShootRpmMargin;
        SmartDashboard.putNumber("Aim/ShootDrumRPM", commanded);
        return commanded;
    }

    /**
     * POV manual tuning: step the shooter drum target by hand and push it
     * straight to the hardware. Deliberately independent of the aim table and
     * the shoot sequence — used to dial in table values by observation.
     */
    private void nudgeManualShooterRpm(double deltaRpm) {
        // Clamp at 0: the drum only ever runs the feeding way, so a negative
        // "target" is meaningless (it would just spin the same direction).
        m_manualShooterRpm = Math.max(0.0, m_manualShooterRpm + deltaRpm);
        shooter.setDrumRpm(m_manualShooterRpm);
        SmartDashboard.putNumber("Tune/ShooterRpm", m_manualShooterRpm);
    }

    /** POV manual tuning: step the hood angle by hand (clamped to travel). */
    private void nudgeManualHoodDeg(double deltaDeg) {
        m_manualHoodDeg = Math.max(0.0,
            Math.min(Constants.Hood.kHoodTravelDegrees, m_manualHoodDeg + deltaDeg));
        hood.setHoodDegrees(m_manualHoodDeg);
        SmartDashboard.putNumber("Tune/HoodDeg", m_manualHoodDeg);
    }

    // Subsystems
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final Intake intake = new Intake();
    private final Hopper hopper = new Hopper();
    private final Shooter shooter = new Shooter();
    private final Hood hood = new Hood();
    private final LED led = new LED();
    private final LimelightVision vision = new LimelightVision(drivetrain, logger.field);
    private final PDHLogger pdh = new PDHLogger(25);
    private final FieldZones fieldZones = new FieldZones(() -> drivetrain.getState().Pose);

    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        // Stream pose + module state to NT (Elastic field widgets read these).
        drivetrain.registerTelemetry(logger::telemeterize);
        configureBindings();
        configureVisionLogging();

        MechanismTriggers.register(intake, hopper, shooter, hood);
        registerButtonNamedCommands();

        // Auto chooser from every auto on the robot (deploy/pathplanner/autos).
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        SmartDashboard.putBoolean("Test Mode", Constants.kTestControlsDefaultOnBoot);

        // Manual POV tuning readouts (see the match-controls POV bindings).
        SmartDashboard.putNumber("Tune/ShooterRpm", m_manualShooterRpm);
        SmartDashboard.putNumber("Tune/HoodDeg", m_manualHoodDeg);

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

    /**
     * One-shot boot homing of the intake pivot: on the first enable it drives
     * to the nearer travel end and re-zeros the virtual trim. Skipped when a
     * home is already stored in Preferences, once it has run this power cycle,
     * or if disabled by kPivotHomeOnBoot.
     */
    private Command intakeBootHome() {
        return Commands.either(
            Commands.runOnce(() -> m_intakeHomed = true)
                .andThen(intake.homeToNearestEnd()),
            Commands.none(),
            () -> Constants.Intake.kPivotHomeOnBoot && !m_intakeHomed && !intake.hasStoredHome());
    }

    /**
     * Intake button (B). Three cases:
     * <ul>
     *   <li>Stowed: deploy and run intake.</li>
     *   <li>Deployed while X is held (outtaking): cancel the outtake hold and
     *       resume intake; do NOT stow.</li>
     *   <li>Deployed and running normally: stow.</li>
     * </ul>
     * Outtake is a hold on X and is refused while stowed.
     */
    private void handleIntakeButton() {
        if (!m_intakeDeployed) {
            m_intakeDeployed = true;
            m_outtakeSuppressed = false;
            intake.setPivotTarget(Constants.Intake.kPivotDeployedRot);
        } else if (outtakeHeld()) {
            m_outtakeSuppressed = true;
            intake.setPivotTarget(Constants.Intake.kPivotDeployedRot);
        } else {
            m_intakeDeployed = false;
            m_outtakeSuppressed = false;
            intake.setPivotTarget(Constants.Intake.kPivotStowedRot);
        }
    }

    /** X is the outtake hold (either pad). Suppressed until X is re-pressed. */
    private boolean outtakeHeld() {
        return driver.x().getAsBoolean() || operator.x().getAsBoolean();
    }

    /** Outtake is actually running: X held and not cancelled by the intake button. */
    private boolean outtaking() {
        return outtakeHeld() && !m_outtakeSuppressed;
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
        }, hood, intake).andThen(intakeBootHome()));
        RobotModeTriggers.teleop().onTrue(Commands.runOnce(() -> {
            hood.setBrakeMode(true);
            intake.setPivotBrakeMode(true);
        }, hood, intake).andThen(intakeBootHome()));

        /* ============ Drive ============ */
        // Aim-toggle state (LT latch): defined up here because the slow-mode
        // bindings below need it — slow mode yields to aim, never fights it.
        Trigger aimActive = new Trigger(this::shootingNow);
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> {
                SmartDashboard.putBoolean("Aim/Active", false);
                SmartDashboard.putBoolean("Aim/AlignedToHub", false);
                // Shoot trigger auto-slows the whole drive to 0.3x so the
                // swerve doesn't sag the drum while a shot is running.
                double scale = shootingNow() ? kShootDriveScale : 1.0;
                SmartDashboard.putBoolean("Aim/ShootSlow", scale < 1.0);
                return drive.withVelocityX(xLimiter.calculate(-driver.getLeftY()) * MaxSpeed * SpeedMultiplier * scale)
                    .withVelocityY(yLimiter.calculate(-driver.getLeftX()) * MaxSpeed * SpeedMultiplier * scale)
                    .withRotationalRate(rotationalLimiter.calculate(-driver.getRightX()) * MaxAngularRate * SpeedMultiplier * scale);
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
        // Fresh recipe instances are built by aimAndShoot() per binding.
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
        // RT: shoot hold = the WHOLE thing: hub heading lock (aimDriveRequest,
        // passing included), hood tracking (AimHood), drums rev to the
        // distance-table RPM, spindle + agitator kick to full feed once the
        // drums are up. Releasing RT ends the lock and stows the hood.
        // Built per call: WPILib forbids reusing a composed Command in a
        // second composition (both pads share the recipe, not the instance).
        driver.rightTrigger().and(matchControls).whileTrue(aimAndShoot());
        operator.rightTrigger().and(matchControls).whileTrue(aimAndShoot());
        driver.rightTrigger().and(matchControls).onTrue(
            Commands.runOnce(() -> shooter.resetFeedLatch()).andThen(intake.shootDip()));
        operator.rightTrigger().and(matchControls).onTrue(
            Commands.runOnce(() -> shooter.resetFeedLatch()).andThen(intake.shootDip()));
        // B: intake button (both pads). Stowed -> deploy + intake; deployed and
        // outtaking -> resume intake (never stows on that press); deployed and
        // running normally -> stow.
        driver.b().and(matchControls).onTrue(Commands.runOnce(this::handleIntakeButton));
        operator.b().and(matchControls).onTrue(Commands.runOnce(this::handleIntakeButton));
        // Rollers follow the latch (25 RPS) whenever deployed and NOT outtaking.
        // Dip/test/auto commands requiring intake supersede while they run, then
        // this resumes. Enabled-only: nothing auto-runs while disabled.
        new Trigger(() -> m_intakeDeployed && !outtaking()).and(matchControls)
            .and(RobotModeTriggers.disabled().negate())
            .whileTrue(intake.runRoller(Constants.Intake.kRollerBaseRPS));
        // Outtake is a HOLD (X), only possible while deployed.
        new Trigger(() -> m_intakeDeployed && outtaking()).and(matchControls)
            .and(RobotModeTriggers.disabled().negate())
            .whileTrue(intake.runRoller(-0.5));
        // LT aim toggle retired 10/22: aiming now happens inside the RT shoot
        // hold (heading lock + hood track + rev + feed in one). LT is free.
        // Releasing aim drops the driver's manual heading nudge so it never
        // carries into normal driving or the next aim session.
        aimActive.onFalse(Commands.runOnce(this::resetAimHeadingOffset));
        // Hood stows when the hold releases (enabled-only: firing this through
        // a disable would schedule a ghost stow that drives the hood on the
        // next enable — outputs are dead in disabled, the command isn't).
        aimActive.onFalse(Commands.either(hood.setAngle(0.0), Commands.none(),
            DriverStation::isEnabled));
        // X: outtake (reverse rollers, both pads) — a HOLD, and disabled while
        // stowed. Each press clears a previous B-cancel so the hold works again.
        driver.x().and(matchControls)
            .onTrue(Commands.runOnce(() -> m_outtakeSuppressed = false));
        operator.x().and(matchControls)
            .onTrue(Commands.runOnce(() -> m_outtakeSuppressed = false));
        operator.a().and(matchControls).whileTrue(drivetrain.applyRequest(() -> brake));
        // Slow 0.3x (both pads) + seed (both pads, match).
        // Match slow yields to aim (see drive section): RB while aimed only
        // scales the aim request's translation.
        operator.rightBumper().and(matchControls).and(aimActive.negate())
            .whileTrue(drivetrain.applyRequest(this::slowDriveRequest));
        operator.leftBumper().and(matchControls)
            .onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        // POV manual tuning (match controls): POV up/down step the shooter
        // drum target by 100 RPM, POV left/right step the hood angle by 2
        // degrees. Direct manual control for dialing in the aim table; it does
        // NOT touch the aim code or the shoot sequence.
        operator.povUp().and(matchControls)
            .onTrue(Commands.runOnce(() -> nudgeManualShooterRpm(100.0)));
        operator.povDown().and(matchControls)
            .onTrue(Commands.runOnce(() -> nudgeManualShooterRpm(-100.0)));
        operator.povLeft().and(matchControls)
            .onTrue(Commands.runOnce(() -> nudgeManualHoodDeg(2.0)));
        operator.povRight().and(matchControls)
            .onTrue(Commands.runOnce(() -> nudgeManualHoodDeg(-2.0)));

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
        operator.rightBumper().and(testControls)
            .whileTrue(intake.runRoller(Constants.Intake.kRollerBaseRPS));
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

    /**
     * Named commands that reproduce the driver button actions for PathPlanner
     * autos:
     * <ul>
     *   <li>{@code IntakeToggle} — exactly what B does (deploy / cancel
     *       outtake / stow), including the deployed latch so the roller
     *       trigger resumes.</li>
     *   <li>{@code ShootStart} — starts the full RT hold in the background
     *       (hub lock, hood track, drum rev, feed when ready, intake dip).
     *       It is INSTANT so a sequential auto can continue to a Wait.</li>
     *   <li>{@code ShootEnd} — cancels the hold, restores the intake, and
     *       stows the hood. Nothing is canceled before this is called.</li>
     * </ul>
     * ShootStart takes the drivetrain (aim lock), so call it after the path
     * has finished; otherwise it will interrupt path following.
     */
    private void registerButtonNamedCommands() {
        // In an auto this must only START the deploy and return immediately.
        // Do not reuse the driver toggle here: the latch state is irrelevant in
        // auto, and a blocking deploy command would hold the sequential group
        // before the path starts. This is a one-shot pivot target write.
        NamedCommands.registerCommand("IntakeToggle", Commands.runOnce(() -> {
            m_intakeDeployed = true;
            m_outtakeSuppressed = false;
            intake.setPivotTarget(Constants.Intake.kPivotDeployedRot);
        }));

        // Instant: launch the persistent hold, then return so the auto can
        // run a Wait before ShootEnd.
        NamedCommands.registerCommand("ShootStart", Commands.runOnce(() -> {
            if (m_autoShootHold != null) {
                m_autoShootHold.cancel();
            }
            if (m_autoShootDip != null) {
                m_autoShootDip.cancel();
                m_autoShootDip = null;
            }
            shooter.resetFeedLatch();
            m_autoShootHold = aimAndShoot();
            CommandScheduler.getInstance().schedule(m_autoShootHold);
            if (m_intakeDeployed) {
                m_autoShootDip = intake.shootDip();
                CommandScheduler.getInstance().schedule(m_autoShootDip);
            }
        }));

        NamedCommands.registerCommand("ShootEnd",
            Commands.runOnce(this::endAutoShoot));
    }

    /** Cancels the auto shoot hold and restores the RT-release state. */
    private void endAutoShoot() {
        if (m_autoShootHold != null) {
            m_autoShootHold.cancel();
            m_autoShootHold = null;
        }
        if (m_autoShootDip != null) {
            m_autoShootDip.cancel();
            m_autoShootDip = null;
        }
        if (m_intakeDeployed) {
            intake.setPivotTarget(Constants.Intake.kPivotDeployedRot);
            intake.setRollerRps(Constants.Intake.kRollerBaseRPS);
        } else {
            intake.setPivotTarget(Constants.Intake.kPivotStowedRot);
        }
        hood.setHoodDegrees(0.0);
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
        // The PathPlanner auto owns its starting pose: each .auto has
        // "resetOdom": true, which resets odometry (and applies the alliance
        // flip) inside PathPlannerAuto.initialize(). Do not wrap it in another
        // resetOdom here — a second reset at schedule time just adds work at
        // the auto-start edge.
        return autoChooser.getSelected();
    }
}