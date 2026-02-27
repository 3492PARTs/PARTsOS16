package frc.robot.constants;

public class TurretConstants {
    public static final int TURRET_MOTOR_ID = 39;
    public static final String CAN_BUS_NAME = "bye";
    /** The turret gear ratio. The total ratio is {@code 40/1}. */
    public static final double TURRET_GEAR_RATIO = (200.0/1.0)*(1.0/20.0)*(4.0/1.0);

    // PID Controller
    public static final double P = 0.3;
    public static final double I = 0;
    public static final double D = 0;
    public static final int PID_THRESHOLD = 1;

    public static final double TURRET_MAX_VELOCITY = 6000;
    public static final double TURRET_MAX_ACCELERATION = 3000;

    // Feedforward
    public static final double S = 0;
    public static final double V = 0;
    public static final double A = 0;
}
