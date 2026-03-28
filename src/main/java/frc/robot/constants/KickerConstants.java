package frc.robot.constants;

import org.parts3492.partslib.PARTsUnit;
import org.parts3492.partslib.PARTsUnit.PARTsUnitType;

public class KickerConstants {
    public enum KickerState {
        IDLE(0),
        DISABLED(0),
        ROLLING(1);

        private final double speed;

        KickerState(double speed) {
            this.speed = speed;
        }

        public double getSpeed() {
            return speed;
        }
    }

    public static final int KICKER_MOTOR_ID = 34;
    public static final double KICKER_GEAR_RATIO = (4.0 / 1.0);
    public static final String CAN_BUS_NAME = "bye";

    public static final PARTsUnit KICKER_WHEEL_RADIUS = new PARTsUnit(1.0, PARTsUnitType.Inch);
}
