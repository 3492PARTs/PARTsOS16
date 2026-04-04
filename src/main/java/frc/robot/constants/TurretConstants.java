package frc.robot.constants;

import org.parts3492.partslib.PARTsUnit;
import org.parts3492.partslib.PARTsUnit.PARTsUnitType;

public class TurretConstants {
    public enum TurretState {
        DISABLED(0),
        IDLE(0),
        TRACKING_HUB(-1),
        TRACKING_CORNER(-1),
        LEFT_CORNER(40),
        RIGHT_CORNER(-40);

        private final double angle;

        TurretState(double angle) {
            this.angle = angle;
        }

        public double getAngle() {
            return angle;
        }
    }

    public static final int TURRET_MOTOR_ID = 39;
    public static final String CAN_BUS_NAME = "bye";
    /** The turret gear ratio. The total ratio is {@code 10/1}. */
    public static final double TURRET_GEAR_RATIO = (200.0 / 20.0) * (4.0 / 1.0);

    // Abs. Encoder

    /**
     * The digital I/O Port of the Through Bore Encoder used for the Turret.
     * <p/>
     * TODO: Get proper DIO port.
     */
    public static final int TURRET_ENCODER_PORT = 0;

    /**
     * Even knowing that the {@link frc.robot.util.PARTsThroughBoreEncoder
     * PARTsThroughBoreEncoder} takes in the angle in degrees, it's still good
     * practice to use units.
     */
    public static final PARTsUnit TURRET_OFFSET_ANGLE = new PARTsUnit(118.9, PARTsUnitType.Angle);

    // PID Controller
    public static final double P = 0.1; //0.3
    public static final double I = 0;
    public static final double D = 0;
    public static final int PID_THRESHOLD = 1;

    // Feedforward
    public static final double S = 0.58061;
    public static final double V = 0;
    public static final double A = 0;

    public static final double TURRET_MAX_VELOCITY = 12000;
    public static final double TURRET_MAX_ACCELERATION = 6000;

    public static final PARTsUnit TURRET_OFFSET_CENTER = new PARTsUnit(8, PARTsUnitType.Inch);
}
