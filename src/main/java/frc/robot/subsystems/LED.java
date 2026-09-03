package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

/**
 * Robot status LEDs on a CTRE CANdle attached to the CANivore bus. Uses the
 * Phoenix 6 {@link CANdle} API: a single solid color or one of the built-in
 * animations (strobe / rainbow) applied to the whole strip on anim slot 0.
 *
 * <p>Device number comes from {@link Constants.old}.
 */
public class LED extends SubsystemBase {
    private final int m_ledCount;

    private final CANdle m_candle;

    private static final RGBWColor kRed = new RGBWColor(255, 0, 0);
    private static final RGBWColor kGold = new RGBWColor(255, 215, 0);
    private static final RGBWColor kGreen = new RGBWColor(0, 255, 0);

    public LED() {
        m_candle = new CANdle(Constants.old.LED.kCANdle, Constants.kCANivoreBus);
        m_ledCount = Constants.LED.kNyLEDCount;

        CANdleConfiguration cfg = new CANdleConfiguration();
        cfg.LED.BrightnessScalar = Constants.LED.kBrightness;
        m_candle.getConfigurator().apply(cfg);
    }

    /** Sets the whole strip to a single solid color. */
    public void setColor(RGBWColor color) {
        m_candle.setControl(
            new SolidColor(Constants.LED.kFirstLED, m_ledCount - 1).withColor(color));
    }

    /** Rainbow across the whole strip. */
    public void setRainbow() {
        m_candle.setControl(new RainbowAnimation(Constants.LED.kFirstLED, m_ledCount - 1));
    }

    /** Strobe the whole strip the given color. */
    public void setStrobe(RGBWColor color) {
        m_candle.setControl(
            new StrobeAnimation(Constants.LED.kFirstLED, m_ledCount - 1).withColor(color));
    }

    /** Solid green = robot armed (teleop/auto). */
    public Command solidGreen() {
        return runOnce(() -> setColor(kGreen));
    }

    /** Gold for teleop. */
    public Command solidGold() {
        return runOnce(() -> setColor(kGold));
    }

    /** Rainbow when idling. */
    public Command rainbow() {
        return runOnce(this::setRainbow);
    }

    /** Red strobe when the robot is disabled. */
    public Command disabledStrobe() {
        return runOnce(() -> setStrobe(kRed));
    }

    /** Blue while an alliance change / error is present (placeholder). */
    public Command allianceError() {
        return runOnce(() -> setColor(new RGBWColor(Color.kBlue)));
    }

    @Override
    public String toString() {
        return "LED(" + Constants.old.LED.kCANdle + ")";
    }
}