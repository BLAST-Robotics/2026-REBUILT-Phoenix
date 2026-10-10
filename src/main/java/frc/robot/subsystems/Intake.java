package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.hardware.MotorConfigs;

/** Roller that pulls spheres in, pivot that raises/lowers it. */
public class Intake extends SubsystemBase {
    private final TalonFX m_roller;
    private final TalonFX m_pivot;

    private double m_pivotTargetRot = 0.0;

    public Intake() {
m_roller = new TalonFX(Constants.old.Intake.kRoller, Constants.kCANivoreBus);
        m_pivot = new TalonFX(Constants.old.Intake.kPivot, Constants.kCANivoreBus);
        CANcoder pivotEncoder = new CANcoder(Constants.old.Intake.kPivotEncoder, Constants.kCANivoreBus);

        MotorConfigs.applyRollerConfig(m_roller, Constants.Intake.kRollerInverted,
            Constants.Intake.kRollerSupplyLimit, Constants.Intake.kPivotStatorLimit);

        MotorConfigs.applyPivotConfig(m_pivot, pivotEncoder,
            Constants.Intake.kPivotInverted, Constants.Intake.kPivotSupplyLimit, Constants.Intake.kPivotStatorLimit,
            Constants.Intake.kPivotTotalReduction,
            Constants.Intake.kPivotReverseSoftLimitRot, Constants.Intake.kPivotForwardSoftLimitRot,
            Constants.Intake.kPivotEncoderOffset);
    }

/** Positive percent pulls spheres in. */
    public void setRollerPercentage(double percent) {
        m_roller.set(percent);
    }

    /** Percent drive. */
    public void setPivotPercentage(double percent) {
        m_pivot.set(percent);
    }

    /** Move pivot to a target output rotation using Motion Magic. */
    public void setPivotTarget(double outputRotations) {
        m_pivotTargetRot = outputRotations;
        m_pivot.setControl(new PositionVoltage(Rotations.of(outputRotations)));
    }

    public double getPivotOutputRotations() {
        return m_pivot.getPosition().getValueAsDouble();
    }

    public boolean atPivotTarget(double outputRotations, double tolerance) {
        return Math.abs(getPivotOutputRotations() - outputRotations) < tolerance;
    }

    public Command deploy() {
        return runOnce(() -> setPivotTarget(Constants.Intake.kPivotDeployedRot))
            .andThen(run(() -> setRollerPercentage(0.4)));
    }

    public Command store() {
        return runOnce(() -> setPivotTarget(Constants.Intake.kPivotStowedRot))
            .andThen(run(() -> setRollerPercentage(0.0)));
    }

    public Command runRoller(double percent) {
        return run(() -> setRollerPercentage(percent));
    }

    @Override
    public void periodic() {
        double pivotRot = getPivotOutputRotations();
        SmartDashboard.putNumber("Intake/PivotRot", pivotRot);
        SmartDashboard.putNumber("Intake/PivotTargetRot", m_pivotTargetRot);
        SmartDashboard.putBoolean("Intake/PivotAtTarget",
            Math.abs(pivotRot - m_pivotTargetRot) < 0.01);
        SmartDashboard.putBoolean("Intake/Deployed",
            Math.abs(pivotRot - Constants.Intake.kPivotDeployedRot) < 0.02);
        SmartDashboard.putNumber("Intake/RollCur", m_roller.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Intake/RollSupplyCur", m_roller.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Intake/RollRPS", m_roller.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Intake/RollOut", m_roller.getDutyCycle().getValueAsDouble());
        SmartDashboard.putNumber("Intake/PivotCur", m_pivot.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Intake/PivotSupplyCur", m_pivot.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Intake/PivotTempC", m_pivot.getDeviceTemp().getValueAsDouble());
    }
}
