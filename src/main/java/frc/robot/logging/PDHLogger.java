package frc.robot.logging;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Reads PDH 2.0 channel currents and publishes them for the dashboard and log. */
public class PDHLogger extends SubsystemBase {
    private final PowerDistribution m_pdh;

    // Cache per-channel current so we don't allocate arrays every loop.
    private final double[] m_channelCurrent = new double[17];
    private int m_loopCount = 0;

    public PDHLogger(int canId) {
        m_pdh = new PowerDistribution(canId, ModuleType.kRev);
    }

    @Override
    public void periodic() {
        // PDH reads are CAN transactions: 5 Hz is plenty, 50 Hz was choking
        // the shared rio bus and starving mechanism feedback.
        if (++m_loopCount % 10 != 0) {
            return;
        }
        for (int i = 0; i < m_channelCurrent.length; i++) {
            m_channelCurrent[i] = m_pdh.getCurrent(i);
            SmartDashboard.putNumber("PDH/Channel/" + i, m_channelCurrent[i]);
        }

        SmartDashboard.putNumber("PDH/TotalCurrent", m_pdh.getTotalCurrent());
        // PDH 2.0 confirmed on CAN id 25: use its own battery voltage read.
        SmartDashboard.putNumber("PDH/Voltage", m_pdh.getVoltage());

        // Log to the loop output file for post-match debugging.
        DataLogManager.log("PDH total current: " + m_pdh.getTotalCurrent()
            + "A @ " + m_pdh.getVoltage() + "V");
    }

    /** Publishes the PDH tab in Elastic/SmartDashboard. */
    public void populateDashboard() {
        SmartDashboard.putData("PDH", m_pdh);
    }
}