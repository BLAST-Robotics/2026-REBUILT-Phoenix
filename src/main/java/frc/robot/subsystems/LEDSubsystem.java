package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.FireAnimation;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
    private final CANdle ledController = new CANdle(0);
    private final CANdleConfiguration ledConfig = new CANdleConfiguration();

    private final CANdle candle;
    private final FireAnimation fireAnimation;
    
    public LEDSubsystem(int candleId) {
        candle = new CANdle(candleId);
        
        CANdleConfiguration config = new CANdleConfiguration();
        candle.getConfigurator().apply(config);

        // Fire animation dynamically creates a pulsing perlin-noise-like effect 
        // from yellowish orange to deep red.
        // It pulses between these colors natively.
        fireAnimation = new FireAnimation(8, 190)
            .withLEDStartIndex(8)
            .withLEDEndIndex(190) // Using withLEDEndIndex directly 
            .withBrightness(1.0)
            .withSparking(0.5) // adjust for more or less "flicker"
            .withCooling(0.4); // adjust for taller/shorter flames
    }

    @Override
    public void periodic() {
        // Run the LEDs during teleop
        if (DriverStation.isTeleopEnabled()) {
            candle.setControl(fireAnimation);
        } else {
            candle.setControl(new com.ctre.phoenix6.controls.EmptyAnimation(0));
        }
    

}
}