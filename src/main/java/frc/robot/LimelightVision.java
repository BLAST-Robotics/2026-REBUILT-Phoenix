package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.Vision;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * Limelight 3 (MegaTag2) localization. The camera is mounted on the shooter,
 * facing straight backwards, and is used for field pose / odometry (NOT direct
 * aiming).
 *
 * <p>The Limelight's camera resolution is set to 640x480 to reduce the load it
 * places on the roboRIO and the network. MegaTag2 pose estimates come from the
 * NT4 {@code botpose_wpiblue_megatag2} table regardless of camera resolution.
 *
 * <p>Each periodic step:
 * <ul>
 *   <li>sends the current gyro orientation so MegaTag2 can fuse it;</li>
 *   <li>reads the MegaTag2 wpiBlue pose estimate;</li>
 *   <li>rejects bad/too-few-tag measurements and large odom jumps;</li>
 *   <li>calls {@link CommandSwerveDrivetrain#addVisionMeasurement} with the
 *       correct FPGA timestamp.</li>
 * </ul>
 */
public class LimelightVision extends SubsystemBase {
    private final CommandSwerveDrivetrain m_drivetrain;

    public LimelightVision(CommandSwerveDrivetrain drivetrain) {
        m_drivetrain = drivetrain;

        // 640x480: set in the Limelight web UI (Camera -> Camera Config ->
        // Stream Resolution). We also keep the standard (single) stream so we are
        // not wasting CPU/bandwidth on picture-in-picture.
        LimelightHelpers.setStreamMode_Standard(Vision.kCameraName);
    }

    @Override
    public void periodic() {
        // Feed the current gyro yaw to MegaTag2 so it can fuse a pose.
        LimelightHelpers.SetRobotOrientation(Vision.kCameraName,
            m_drivetrain.getState().Pose.getRotation().getDegrees(),
            0.0, 0.0, 0.0, 0.0, 0.0);

        LimelightHelpers.PoseEstimate estimate =
            LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(Vision.kCameraName);

        if (estimate == null || estimate.tagCount < Vision.kMinTagCount) {
            return;
        }

        // Sanity guard: don't let vision teleport the robot.
        Pose2d measured = estimate.pose;
        if (measured.getTranslation().getDistance(
                m_drivetrain.getState().Pose.getTranslation())
            > Vision.kMaxPoseError) {
            return;
        }

        // estimate.timestampSeconds is the FPGA time of the capture.
        m_drivetrain.addVisionMeasurement(measured, estimate.timestampSeconds);
    }
}