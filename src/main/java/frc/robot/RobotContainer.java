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
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.commands.AimAndShoot;
import frc.robot.commands.ShootOnTheMove;
import frc.robot.generated.TunerConstants;
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

    // Subsystems
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final Intake intake = new Intake();
    private final Hopper hopper = new Hopper();
    private final Shooter shooter = new Shooter();
    private final Hood hood = new Hood();
    private final LED led = new LED();
    private final LimelightVision vision = new LimelightVision(drivetrain);
    private final PDHLogger pdh = new PDHLogger(16); // PDH 2.0 CAN id (todo: confirm)

    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        configureBindings();
        configureVisionLogging();

        MechanismTriggers.register(intake, hopper, shooter, hood);

        // Auto chooser from every auto on the robot (deploy/pathplanner/autos).
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

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

        driver.rightBumper().whileTrue(drivetrain.applyRequest(() ->
            drive.withVelocityX(xLimiter.calculate(-driver.getLeftY()) * MaxSpeed * SpeedMultiplier * 0.3)
                .withVelocityY(yLimiter.calculate(-driver.getLeftX()) * MaxSpeed * SpeedMultiplier * 0.3)
                .withRotationalRate(rotationalLimiter.calculate(-driver.getRightX()) * MaxAngularRate * SpeedMultiplier * 0.3)
        ));

        driver.a().whileTrue(drivetrain.applyRequest(() -> brake));
        driver.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))
        ));
        driver.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        // SysId (keep for characterization), still reachable if needed.
        driver.back().and(driver.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driver.back().and(driver.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driver.start().and(driver.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driver.start().and(driver.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        /* ============ Mechanisms (operator) ============ */
        // Intake
        operator.rightBumper().whileTrue(intake.runRoller(1.0));          // run rollers in
        operator.leftBumper().whileTrue(intake.runRoller(-0.5));          // reverse rollers (clear jam)
        operator.a().whileTrue(intake.deploy());                          // deploy intake
        operator.b().whileTrue(intake.store());                           // stow intake
        operator.povUp().whileTrue(intake.run(() -> intake.setPivotPercentage(0.3)));   // pivot up
        operator.povDown().whileTrue(intake.run(() -> intake.setPivotPercentage(-0.3))); // pivot down

        // Hopper / agitator
        operator.y().whileTrue(hopper.runForward());        // feed spheres forward
        operator.x().whileTrue(hopper.runReverse());        // clear jam

        // Shooter
        operator.rightTrigger().whileTrue(shooter.revToRpm(4000.0));      // rev shooter (tune)
        operator.leftTrigger().whileTrue(shooter.runSpindle(80.0));       // run spindle/fire
        operator.povRight().whileTrue(shooter.revToRpm(6000.0));          // high setpoint
        operator.povLeft().whileTrue(shooter.revToRpm(3000.0));           // low setpoint

        // Hood
        operator.rightTrigger().whileTrue(hood.setAngle(35.0));           // shoot angle (tune)

        // Aiming (uses Limelight pose -> interpolating tables).
        operator.povUp().whileTrue(new AimAndShoot(drivetrain, hood, shooter));

        // Shoot on the move: keeps aiming at the target from the live pose while
        // the robot is driving, firing through the spindle continuously. Hold to
        // keep re-aiming as you travel.
        operator.start().whileTrue(new ShootOnTheMove(drivetrain, hood, shooter));

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