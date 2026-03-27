package frc.robot.constants;
import frc.robot.util.Hub.Targets;

import org.parts3492.partslib.PARTsUnit;
import org.parts3492.partslib.PARTsUnit.PARTsUnitType;

public class ShooterConstants {
    public enum ShooterState {
        IDLE(0),
        DISABLED(0),
        SHOOTING(3500),
        MANUAL(4000);

        private final double rpm;

        ShooterState(double rpm) {
            this.rpm = rpm;
        }

        public double getRPM() {
            return rpm;
        }

        public static double getZoneRPM(Targets zone) {
            if (zone == null) {
                return 0;
            }
            switch (zone) {
                case BEHIND_HUB:
                    return 3000;
                case TRENCH:
                    return 3400;
                case ZONE1:
                    return 3000 - 150;
                case ZONE2:
                    return 3200 - 150;
                case ZONE3:
                    return 3400 - 150;
                case ZONE4:
                    return 3600 - 150;
                case ZONE5:
                    return 3800 - 150;
                case ZONE6:
                    return 4000 - 30;
                default:
                    return 0;
            }
        }
    }

    public static final int LEFT_MOTOR_ID = 33;
    public static final int RIGHT_MOTOR_ID = 35;
    public static final String CAN_BUS_NAME = "bye";

    public static final PARTsUnit SHOOTER_WHEEL_RADIUS = new PARTsUnit(1.5, PARTsUnitType.Inch);
    // TODO: Get actual wheel weight.
    public static final PARTsUnit SHOOTER_WEEL_WEIGHT = new PARTsUnit(4, PARTsUnitType.Pound);

    // PID Controller
    public static final double P = 0.0005;
    public static final double I = 0;
    public static final double D = 0;
    public static final int PID_THRESHOLD = 50;

    // Feedforward
    public static final double S = 0.20465;
    public static final double V = 0.50077;
    public static final double A = 0.50077;
}
