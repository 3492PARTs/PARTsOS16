package frc.robot.constants;

import org.parts3492.partslib.PARTsUnit;
import org.parts3492.partslib.PARTsUnit.PARTsUnitType;

public class HopperConstants {
    public enum HopperState {
        IDLE(0, 0),
        DISABLED(0, 0),
        ROLLING(.8, 1300),
        REVERSE(-.8, -1300);
    
        private final double speed;

        private final double rpm;

        HopperState(double speed, double rpm) {
            this.speed = speed;
            this.rpm = rpm;
        }

        public double getSpeed() {
            return speed;
        }

        public double getRPM() {
            return rpm;
        }
    }

    public static final double P = 0.005;
    public static final double I = 0;
    public static final double D = 0;
    public static final int PID_THRESHOLD = 25;

    public static final double S = 0.10288;
    public static final double V = 4.5;
    public static final double A = 0.094641;

    public static final int HOPPER_MOTOR_ID = 37;
    public static final double HOPPER_GEAR_RATIO = (4.0/1.0);
    public static final String CAN_BUS_NAME = "bye";
    public static final PARTsUnit HOPPER_ROLLER_RADIUS = new PARTsUnit(0.625, PARTsUnitType.Inch);
}
