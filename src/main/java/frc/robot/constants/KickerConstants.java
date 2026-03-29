package frc.robot.constants;

import org.parts3492.partslib.PARTsUnit;
import org.parts3492.partslib.PARTsUnit.PARTsUnitType;

public class KickerConstants {
    public enum KickerState {
        IDLE(0),
        DISABLED(0),
        ROLLING(500);

        private final double RPM;

        KickerState(double RPM) {
            this.RPM = RPM;
        }

        public double getRPM() {
            return RPM;
        }
    }

    public static final int KICKER_MOTOR_ID = 34;
    public static final double KICKER_GEAR_RATIO = (4.0 / 1.0);
    public static final String CAN_BUS_NAME = "bye";

    public static final PARTsUnit KICKER_WHEEL_RADIUS = new PARTsUnit(1.125, PARTsUnitType.Inch);

    // PID Controller
    public static final double P = 0;
    public static final double I = 0;
    public static final double D = 0;
    public static final int PID_THRESHOLD = 100;

    // Feedforward
    public static final double S = 0;
    public static final double V = 0;
    public static final double A = 0;
}
