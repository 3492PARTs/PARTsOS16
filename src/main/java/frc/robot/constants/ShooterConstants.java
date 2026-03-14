package frc.robot.constants;

import org.parts3492.partslib.PARTsUnit;
import org.parts3492.partslib.PARTsUnit.PARTsUnitType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.util.Hub.Targets;

public class ShooterConstants {
    public enum ShooterState {
        IDLE(0),
        DISABLED(0),
        CHARGING(3500),
        SHOOTING(3500),
        MANUAL(getZoneRPM(Targets.ZONE6));

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
                case ZONE1:
                    return 3000;
                case ZONE2:
                    return 3300;
                case ZONE3:
                    return 3400;
                case ZONE4:
                    return 3600;
                case ZONE5:
                    return 3800;
                case ZONE6:
                    return 4000;
                default:
                    return 0;
            }
        }

        /**
         * K = Distance (Meters),
         * <p>
         * V = RPM
         * 
         * Make sure to use the gotten distance from
         * {@link frc.robot.util.Trench#getDistance(Pose2d, Pose2d)
         * Trench.getDistance()} to properly calculate the right RPM.
         */
        public static InterpolatingDoubleTreeMap rpmTable = initRpmTable();

        /**
         * Creates and populates the {@link #rpmTable RPM Table} with the correct position-to-rpm values based off of the current zone code.<p>
         * This function should automatically get called at runtime, so there's no need to expose this function.
         */
        private static InterpolatingDoubleTreeMap initRpmTable() {
            InterpolatingDoubleTreeMap table = new InterpolatingDoubleTreeMap();

            // End of Deadzone / Start of Zone 1
            table.put(PARTsUnit.InchesToMeters.apply(8.0), 3000.0);

            /*
             * I don't think that we need to populate the table with these calcs, but
             * they're here just in case we do.
             * Further testing is required though.
             */
            // table.put(PARTsUnit.InchesToMeters.apply(10.0), 3300.0);
            // table.put(PARTsUnit.InchesToMeters.apply(11.5), 3400.0);
            // table.put(PARTsUnit.InchesToMeters.apply(13.0), 3600.0);
            // table.put(PARTsUnit.InchesToMeters.apply(15.0), 3800.0);
            // table.put(PARTsUnit.InchesToMeters.apply(17.0), 4000.0);

            // Zone 6
            table.put(PARTsUnit.InchesToMeters.apply(19.0), 4000.0);

            return table;
        }
    }

    public static final int LEFT_MOTOR_ID = 33;
    public static final int RIGHT_MOTOR_ID = 35;
    public static final String CAN_BUS_NAME = "bye";
    // public static final boolean SHOOT_DEBUG = false;

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
