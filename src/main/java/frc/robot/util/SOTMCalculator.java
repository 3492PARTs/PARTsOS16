package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.constants.ShooterConstants.ShooterState;
import frc.robot.util.Hub.Targets;

public class SOTMCalculator {
    /**
     * Key = Distance (Meters)
     * <p>
     * Value = RPM
     * 
     * Make sure to use the gotten distance from
     * {@link frc.robot.util.Trench#getDistance(Pose2d, Pose2d)
     * Trench.getDistance()} to properly calculate the right RPM.
     */
    private static InterpolatingDoubleTreeMap rpmTable = initRpmTable();

    /**
     * Key = Distance (Meters)
     * <p>
     * Value = Time of Fuel Flight
     * 
     * Make sure to use the gotten distance from
     * {@link frc.robot.util.Trench#getDistance(Pose2d, Pose2d)
     * Trench.getDistance()} to properly calculate the right RPM.
     */
    private static InterpolatingDoubleTreeMap tofTable = initTofTable();

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
        // ? It's a little bit lower (0.1 feet in meters) than the actual deadzone just
        // in case the robot is slightly in the deadzone, might have to be removed.
        table.put(Targets.DEADZONE.getRadius() - (0.03048), 0.0);
        // End of Deadzone / Start of Zone 1
        table.put(Targets.DEADZONE.getRadius(), ShooterState.getZoneRPM(Targets.ZONE1));

        /*
         * I don't think that we need to populate the table with these zones, but
         * they're here just in case we do.
         * Further testing is required though.
         * It seems to work fine without these values in sim.
         */
        table.put(Targets.ZONE1.getRadius(), ShooterState.getZoneRPM(Targets.ZONE2));
        table.put(Targets.ZONE2.getRadius(), ShooterState.getZoneRPM(Targets.ZONE3));
        table.put(Targets.ZONE3.getRadius(), ShooterState.getZoneRPM(Targets.ZONE4));
        table.put(Targets.ZONE4.getRadius(), ShooterState.getZoneRPM(Targets.ZONE5));
        table.put(Targets.ZONE5.getRadius(), ShooterState.getZoneRPM(Targets.ZONE6));

        // Zone 6
        table.put(Targets.ZONE6.getRadius(), ShooterState.getZoneRPM(Targets.ZONE6));

        return table;
    }

    /**
     * Creates and populates the {@link #tofTable ToF (Time of Flight) Table} with
     * the correct
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
     * Gets the RPM from the distance to the goal pose using the {@link #rpmTable
     * RPM Table}.
     * 
     * @param robotPose The current pose of the robot, used to calculate the
     *                  distance to the goal.
     * @return The RPM corresponding to the distance.
     */
    public static double getRPMToGoal(Pose2d robotPose, Pose2d goalPose) {
        double distance = Trench.getDistance(robotPose, goalPose);
        return rpmTable.get(distance);
    }

    /**
     * Gets the ToF from the distance to the known hub using the {@link #tofTable
     * ToF Table}.
     * 
     * @param robotPose The current pose of the robot, used to calculate the
     *                  distance to the hub.
     * @return The ToF corresponding to the distance.
     */
    public static double getFlightTimeToGoal(Pose2d robotPose, Pose2d goalPose) {
        double distance = Trench.getDistance(robotPose, goalPose);
        return tofTable.get(distance);
    }

    /**
     * Calculates the target robot pose based off the pose, velocity, and the calculated {@link #getFlightTimeToGoal(Pose2d, Pose2d) Time of Flight}. 
     * @param robotPose The supplied robot pose.
     * @param robotVelocity The supplied robot velocity.
     * @return The calculated robot pose.
     */
    public static Pose2d getTargetPose(Pose2d robotPose, Transform2d robotVelocity) {
        Pose2d pose = new Pose2d();
        double tof = getFlightTimeToGoal(robotPose, Field.getAllianceHubPose());

        Transform2d tofPose = new Transform2d(robotVelocity.getX() * tof,
            robotVelocity.getY() * tof,
            new Rotation2d());
        
        pose = robotPose.plus(tofPose);
        return pose;
    }
}
