package frc.robot.logging;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Power Distribution Hub 2.0 monitoring. The PDH 2.0 is the ONLY device on the
 * roboRIO CAN bus (everything else moved to the CANivore). This periodically
 * reads every channel's current and voltage and publishes them for the Elastic
 * dashboard and to a log file for debugging.
 *
 * <p>Channels: 0-15 are the 16 breaker-backed channels (REV PDH). Channel 16 is
 * the dedicated CAN bus channel on the PDH 2.0.
 */
public class PDHLogger extends SubsystemBase {
    private final PowerDistribution m_pdh;

    // Cache per-channel current so we don't allocate arrays every loop.
    private final double[] m_channelCurrent = new double[17];

    public PDHLogger(int canId) {
        // PDH 2.0 on the roboRIO internal bus.
        m_pdh = new PowerDistribution(canId, ModuleType.kRev);
    }

    @Override
    public void periodic() {
        for (int i = 0; i < m_channelCurrent.length; i++) {
            m_channelCurrent[i] = m_pdh.getCurrent(i);
            SmartDashboard.putNumber("PDH/Channel/" + i, m_channelCurrent[i]);
        }

        SmartDashboard.putNumber("PDH/TotalCurrent", m_pdh.getTotalCurrent());
        SmartDashboard.putNumber("PDH/Voltage", m_pdh.getVoltage());

        // Log to the loop output file for historical review (Elastic reads NT,
        // but the log file is useful for post-match debugging).
        DataLogManager.log("PDH total current: " + m_pdh.getTotalCurrent()
            + "A @ " + m_pdh.getVoltage() + "V");
    }

    /** Publishes the PDH tab in Elastic/SmartDashboard. */
    public void populateDashboard() {
        SmartDashboard.putData("PDH", m_pdh);
    }
}