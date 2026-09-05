package frc.robot.logging;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.FieldConstants;

/** Publishes which field zone the robot is in (blue/neutral/red) over NetworkTables. */
public class FieldZones extends SubsystemBase {
    public static final String kBlueZone = "Blue Alliance";
    public static final String kNeutralZone = "Neutral Zone";
    public static final String kRedZone = "Red Alliance";

    private final Supplier<Pose2d> m_pose;

    public FieldZones(Supplier<Pose2d> pose) {
        m_pose = pose;
    }

    public static String zoneFor(double x) {
        if (x < FieldConstants.kHubDistFromWall) {
            return kBlueZone;
        }
        if (x > FieldConstants.kFieldLength - FieldConstants.kHubDistFromWall) {
            return kRedZone;
        }
        return kNeutralZone;
    }

    @Override
    public void periodic() {
        Pose2d pose = m_pose.get();
        SmartDashboard.putString("Field/Zone", zoneFor(pose.getX()));
        SmartDashboard.putNumber("Field/RobotX", pose.getX());
        SmartDashboard.putNumber("Field/RobotY", pose.getY());
        SmartDashboard.putNumber("Field/RobotDeg", pose.getRotation().getDegrees());
    }
}