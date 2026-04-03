package frc.robot.util;

import org.parts3492.partslib.PARTsUnit;
import org.parts3492.partslib.PARTsUnit.PARTsUnitType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.constants.TurretConstants;

public class SOTMCalculator {

    public static class PoseTof {
        Pose2d pose;
        double tof;
        public PoseTof (Pose2d pose, double tof) {
            this.pose = pose;
            this.tof = tof;
        }
    }
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
        table.put(new PARTsUnit(7.6, PARTsUnitType.Foot).to(PARTsUnitType.Meter) - TurretConstants.TURRET_OFFSET_CENTER.to(PARTsUnitType.Meter) - (0.03), 0.0);
        table.put(new PARTsUnit(7.6, PARTsUnitType.Foot).to(PARTsUnitType.Meter) - TurretConstants.TURRET_OFFSET_CENTER.to(PARTsUnitType.Meter), 2525.0);
        table.put(new PARTsUnit(9.4375, PARTsUnitType.Foot).to(PARTsUnitType.Meter) - TurretConstants.TURRET_OFFSET_CENTER.to(PARTsUnitType.Meter), 2550.0);
        table.put(new PARTsUnit(12.4375, PARTsUnitType.Foot).to(PARTsUnitType.Meter) - TurretConstants.TURRET_OFFSET_CENTER.to(PARTsUnitType.Meter), 2825.0);
        table.put(new PARTsUnit(15.4375, PARTsUnitType.Foot).to(PARTsUnitType.Meter) - TurretConstants.TURRET_OFFSET_CENTER.to(PARTsUnitType.Meter), 3125.0);
        table.put(new PARTsUnit(18.4375, PARTsUnitType.Foot).to(PARTsUnitType.Meter) - TurretConstants.TURRET_OFFSET_CENTER.to(PARTsUnitType.Meter), 3400.0);
        table.put(new PARTsUnit(21.4375, PARTsUnitType.Foot).to(PARTsUnitType.Meter) - TurretConstants.TURRET_OFFSET_CENTER.to(PARTsUnitType.Meter), 3700.0);
        table.put(new PARTsUnit(24.4375, PARTsUnitType.Foot).to(PARTsUnitType.Meter) - TurretConstants.TURRET_OFFSET_CENTER.to(PARTsUnitType.Meter), 3950.0);
        table.put(new PARTsUnit(26.8125, PARTsUnitType.Foot).to(PARTsUnitType.Meter) - TurretConstants.TURRET_OFFSET_CENTER.to(PARTsUnitType.Meter), 4250.0);

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
        table.put(new PARTsUnit(7.6, PARTsUnitType.Foot).to(PARTsUnitType.Meter) - TurretConstants.TURRET_OFFSET_CENTER.to(PARTsUnitType.Meter) - (0.03), 0.0);
        table.put(new PARTsUnit(7.6, PARTsUnitType.Foot).to(PARTsUnitType.Meter) - TurretConstants.TURRET_OFFSET_CENTER.to(PARTsUnitType.Meter), 0.90);
        table.put(new PARTsUnit(9.4375, PARTsUnitType.Foot).to(PARTsUnitType.Meter) - TurretConstants.TURRET_OFFSET_CENTER.to(PARTsUnitType.Meter), 0.91);
        table.put(new PARTsUnit(12.4375, PARTsUnitType.Foot).to(PARTsUnitType.Meter) - TurretConstants.TURRET_OFFSET_CENTER.to(PARTsUnitType.Meter), 1.03);
        table.put(new PARTsUnit(15.4375, PARTsUnitType.Foot).to(PARTsUnitType.Meter) - TurretConstants.TURRET_OFFSET_CENTER.to(PARTsUnitType.Meter), 1.12);
        table.put(new PARTsUnit(18.4375, PARTsUnitType.Foot).to(PARTsUnitType.Meter) - TurretConstants.TURRET_OFFSET_CENTER.to(PARTsUnitType.Meter), 1.21);
        table.put(new PARTsUnit(21.4375, PARTsUnitType.Foot).to(PARTsUnitType.Meter) - TurretConstants.TURRET_OFFSET_CENTER.to(PARTsUnitType.Meter), 1.29);
        table.put(new PARTsUnit(24.4375, PARTsUnitType.Foot).to(PARTsUnitType.Meter) - TurretConstants.TURRET_OFFSET_CENTER.to(PARTsUnitType.Meter), 1.43);
        table.put(new PARTsUnit(26.8125, PARTsUnitType.Foot).to(PARTsUnitType.Meter) - TurretConstants.TURRET_OFFSET_CENTER.to(PARTsUnitType.Meter), 1.48);

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
    public static Pose2d getTargetPose(Pose2d robotPose, Transform2d robotVelocity, double tof) {
        Pose2d pose = new Pose2d();

        Transform2d tofPose = new Transform2d(
            robotVelocity.getX() * tof,
            robotVelocity.getY() * tof,
            new Rotation2d()
        );

        pose = robotPose.plus(tofPose);
        return pose;
    }

    public static Pose2d collapsePose(Pose2d robotPose, Transform2d robotVelocity) {
        Pose2d temp  = robotPose;
        for (int i = 0; i < 5; i ++) {
            double tof = getFlightTimeToGoal(temp, Field.getAllianceHubPose());
            temp = getTargetPose(robotPose, robotVelocity, tof);
        }
        
        return temp;
    }
}
