package frc.robot.constants;

import org.parts3492.partslib.PARTsUnit;
import org.parts3492.partslib.PARTsUnit.PARTsUnitType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.util.Field;
import frc.robot.util.Hub.Targets;
import frc.robot.util.Trench;

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

        /**
         * Key = Distance (Meters)
         * <p>
         * Value = RPM
         * 
         * Make sure to use the gotten distance from
         * {@link frc.robot.util.Trench#getDistance(Pose2d, Pose2d) Trench.getDistance()} to properly calculate the right RPM.
         */
        public static InterpolatingDoubleTreeMap rpmTable = initRpmTable();

        /**
         * Key = Distance (Meters)
         * <p>
         * Value = Time of Fuel Flight
         * 
         * Make sure to use the gotten distance from
         * {@link frc.robot.util.Trench#getDistance(Pose2d, Pose2d) Trench.getDistance()} to properly calculate the right RPM.
         */
        public static InterpolatingDoubleTreeMap tofTable = initTofTable();

        /**
         * Creates and populates the {@link #rpmTable RPM Table} with the correct
         * position-to-rpm values based off of the current zone code.
         * <p>
         * This function should automatically get called at runtime, so there's no need
         * to expose this function.
         */
        private static InterpolatingDoubleTreeMap initRpmTable() {
            InterpolatingDoubleTreeMap table = new InterpolatingDoubleTreeMap();

            // Cut off the shooter in the deadzone.
            //? It's a little bit lower (0.1 feet in meters) than the actual deadzone just in case the robot is slightly in the deadzone, might have to be removed.
            table.put(Targets.DEADZONE.getRadius() - (0.03048), 0.0);
            // End of Deadzone / Start of Zone 1
            table.put(Targets.DEADZONE.getRadius(), getZoneRPM(Targets.ZONE1));

            /*
             * I don't think that we need to populate the table with these zones, but
             * they're here just in case we do.
             * Further testing is required though.
             * It seems to work fine without these values in sim.
             */
            table.put(Targets.ZONE1.getRadius(), getZoneRPM(Targets.ZONE2));
            table.put(Targets.ZONE2.getRadius(), getZoneRPM(Targets.ZONE3));
            table.put(Targets.ZONE3.getRadius(), getZoneRPM(Targets.ZONE4));
            table.put(Targets.ZONE4.getRadius(), getZoneRPM(Targets.ZONE5));
            table.put(Targets.ZONE5.getRadius(), getZoneRPM(Targets.ZONE6));

            // Zone 6
            table.put(Targets.ZONE6.getRadius(), getZoneRPM(Targets.ZONE6));

            return table;
        }

        /**
         * Creates and populates the {@link #tofTable ToF (Time of Flight) Table} with the correct
         * position-to-ToF values based off of the current zone code.
         * <p>
         * This function should automatically get called at runtime, so there's no need
         * to expose this function.
         */
        private static InterpolatingDoubleTreeMap initTofTable() {
            InterpolatingDoubleTreeMap table = new InterpolatingDoubleTreeMap();

            // 0 Little less than deadzone to make sure it's set to zero like it's supposed to be.
            table.put(Targets.DEADZONE.getRadius() - (0.03048), Targets.DEADZONE.getTimeOfFlight());
            table.put(Targets.DEADZONE.getRadius(), Targets.ZONE1.getTimeOfFlight());

            // Min
            table.put(Targets.ZONE1.getRadius(), Targets.ZONE2.getTimeOfFlight());
            table.put(Targets.ZONE2.getRadius(), Targets.ZONE3.getTimeOfFlight());
            table.put(Targets.ZONE3.getRadius(), Targets.ZONE4.getTimeOfFlight());
            table.put(Targets.ZONE4.getRadius(), Targets.ZONE5.getTimeOfFlight());
            table.put(Targets.ZONE5.getRadius(), Targets.ZONE6.getTimeOfFlight());

            // Max
            table.put(Targets.ZONE6.getRadius(), Targets.ZONE6.getTimeOfFlight());

            return table;
        }

        /**
         * Gets the RPM from the distance to the known hub using the {@link #rpmTable RPM Table}.
         * 
         * @param robotPose The current pose of the robot, used to calculate the
         *                  distance to the hub.
         * @return The RPM corresponding to the distance.
         */
        public static double getRPMFromDistanceToHub(Pose2d robotPose) {
            Pose2d hubPose = Field.getAllianceHubPose();

            double distance = Trench.getDistance(robotPose, hubPose);
            return rpmTable.get(distance);
        }

        /**
         * Gets the ToF from the distance to the known hub using the {@link #tofTable ToF Table}.
         * 
         * @param robotPose The current pose of the robot, used to calculate the
         *                  distance to the hub.
         * @return The ToF corresponding to the distance.
         */
        public static double getTofFromDistanceToHub(Pose2d robotPose) {
            Pose2d hubPose = Field.getAllianceHubPose();

            double distance = Trench.getDistance(robotPose, hubPose);
            return tofTable.get(distance);
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
