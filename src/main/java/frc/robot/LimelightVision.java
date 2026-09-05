package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.Vision;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * Limelight 3 (MegaTag2) localization for field pose / odometry. The camera
 * faces straight backwards off the shooter and is not used for direct aiming.
 */
public class LimelightVision extends SubsystemBase {
    private final CommandSwerveDrivetrain m_drivetrain;

    public LimelightVision(CommandSwerveDrivetrain drivetrain) {
        m_drivetrain = drivetrain;

        // Single (standard) stream to save CPU/bandwidth.
        LimelightHelpers.setStreamMode_Standard(Vision.kCameraName);
    }

    @Override
    public void periodic() {
        // Send the current gyro yaw so MegaTag2 can fuse a pose.
        LimelightHelpers.SetRobotOrientation(Vision.kCameraName,
            m_drivetrain.getState().Pose.getRotation().getDegrees(),
            0.0, 0.0, 0.0, 0.0, 0.0);

        LimelightHelpers.PoseEstimate estimate =
            LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(Vision.kCameraName);

        if (estimate == null || estimate.tagCount < Vision.kMinTagCount) {
            return;
        }

        // Don't let vision teleport the robot.
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