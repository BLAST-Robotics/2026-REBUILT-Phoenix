package frc.robot;

public class Constants {
  public static final class IntakeConstants
  {
    public static final int PIVOT_ID = 18;
    public static final int ROLLER_ID = 16;
    public static final int AGITATOR_ID = 15;

    public static final double PIVOT_GEAR_RATIO = 52.5;
    public static final double ROLLER_GEAR_RATIO = 2.0;

    public static final double PIVOT_UP_POWER = 0.2;
    public static final double PIVOT_DOWN_POWER = -0.2; // Reversed as requested
    public static final double PIVOT_RAMP_TIME = 0.5; // Seconds to ramp up

    public static final double ROLLER_DEFAULT_RPM = 1500.0;
    public static final double ROLLER_REVERSE_VOLTAGE = -6.0;
    
    public static final double AGITATOR_VOLTS = 0;//11.0;
    public static final double AGITATOR_REVERSE_VOLTS = 0; //-11.0;
    public static final boolean AGITATOR_INVERT = true;
  }
 public static final class OperatorConstants
  {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;

    // Control Scheme
    public static final boolean DRIVER_PIVOT = true;
    public static final boolean OPERATOR_PIVOT = true;
    public static final boolean DRIVER_STEER = true;
    public static final boolean OPERATOR_STEER = false;
    public static final boolean DRIVER_INTAKE = true;
    public static final boolean OPERATOR_INTAKE = true;
    public static final boolean DRIVER_SHOOT = true;
    public static final boolean OPERATOR_SHOOT = true;

    // Set this to true to use only one controller, false to use two.
    public static final boolean SINGLE_CONTROLLER_MODE = false;

    // Set this to true to completely disable auto-aim features.
    public static final boolean AUTO_AIM_DISABLED = true;

    // Subsystem Initialization Toggles
    public static final boolean SWERVE_ENABLED = true;
    public static final boolean INTAKE_ENABLED = true;
    public static final boolean SHOOTER_ENABLED = true;
    public static final boolean VISION_ENABLED = true;
    public static final boolean AGITATOR_ENABLED = false;

    // CAN IDs
    public static final int CANDLE_ID = 30;

    // Joystick Deadband
    public static final double LEFT_X_DEADBAND  = 0.03;
    public static final double LEFT_Y_DEADBAND  = 0.03;
    public static final double RIGHT_X_DEADBAND = 0.03;
    public static final double TURN_CONSTANT    = 6;
  }
  public static final class ShooterConstants
  {
    public static final int RIGHT_MOTOR_ID = 9;
    public static final int LEFT_MOTOR_ID = 10;
    public static final int INDEXER_MOTOR_ID = 17;

    public static final double SHOOTER_kV = 0.12; // Starting value, tune as needed
    public static final double SHOOTER_kP = 0.11; // Starting value, tune as needed

    public static final double VELOCITY_THRESHOLD = 1.0; // RPS threshold for isUpToSpeed

    public static final double INDEXER_POWER = 0.8; // Open loop power for indexer
    public static final double INDEXER_REVERSE_POWER = -0.8;

    public static final double SHOOTER_DEFAULT_RPM = 2500.0;
    public static final double SHOOTER_REVERSE_RPM = -1000.0;
  }
}
