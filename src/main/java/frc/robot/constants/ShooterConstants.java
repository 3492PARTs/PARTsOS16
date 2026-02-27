package frc.robot.constants;

import org.parts3492.partslib.PARTsUnit;
import org.parts3492.partslib.PARTsUnit.PARTsUnitType;

import frc.robot.util.Hub.Targets;

public class ShooterConstants {
    public enum ShooterState {
        IDLE(0),
        DISABLED(0),
        CHARGING(3500),
        SHOOTING(3500);

        private final double rpm;

        ShooterState(double rpm) {
            this.rpm = rpm;
        }

        public double getRPM() {
            return rpm;
        }

        public double getZoneRPM(Targets zone) {
            if (zone == null) {
                return 0;
            }
            switch (zone) {
                case ZONE1:
                    return 3000;
                case ZONE2:
                    return 3200;
                case ZONE3:
                    return 3400;
                case ZONE4:
                    return 3600;
                default:
                    return 0;
            }
        }
    }

    public static final int LEFT_MOTOR_ID = 33;
    public static final int RIGHT_MOTOR_ID = 35;
    public static final String CAN_BUS_NAME = "bye";
    public static final boolean SHOOT_DEBUG = false;

    public static final PARTsUnit SHOOTER_WHEEL_RADIUS = new PARTsUnit(1.5, PARTsUnitType.Inch);
    // TODO: Get actual wheel weight.
    public static final PARTsUnit SHOOTER_WEEL_WEIGHT = new PARTsUnit(4, PARTsUnitType.Pound);

    // PID Controller
    public static final double P = 0.0005;
    public static final double I = 0;
    public static final double D = 0;
    public static final int PID_THRESHOLD = 50;

    // Feedforward
    public static final double S = 0.041561;
    public static final double V = 0.48125;
    public static final double A = 0.019433;
}
