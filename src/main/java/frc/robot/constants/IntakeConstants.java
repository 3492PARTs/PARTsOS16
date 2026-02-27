package frc.robot.constants;

public class IntakeConstants {
    public enum IntakeState {
        IDLE(0, 0),
        DISABLED(0, 0),
        INTAKING(0.75, 190),
        OUTTAKING(-0.75, 190),
        SHOOTING(0, 90),
        HOME(0, 0);

        private double speed;
        private double angle;

        private IntakeState(double speed, double angle) {
            this.speed = speed;
            this.angle = angle;
        }

        public double getSpeed() {
            return speed;
        }

        public double getAngle() {
            return angle;
        }
    }

    public static final int INTAKE_MOTOR_ID = 36;
    public static final int PIVOT_MOTOR_ID = 38;
    public static final String CAN_BUS_NAME = "bye";

    /** The pivot gear ratio. The total is {@code 36/1}. */
    public static final double PIVOT_GEAR_RATIO = (12.0 / 1.0) * (3.0 / 1.0);

    // PID Controller
    public static final double P = 0.07;
    public static final double I = 0;
    public static final double D = 0;
    public static final int PID_THRESHOLD = 1;

    public static final double INTAKE_MAX_VELOCITY = 200;
    public static final double INTAKE_MAX_ACCELERATION = 400;

    // Feedforward
    public static final double S = 0;
    public static final double V = 0;
    public static final double A = 0;
}
