package frc.robot.util;

import org.parts3492.partslib.PARTsUnit.PARTsUnitType;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.constants.FieldConstants;

public class Hub {
    private static Pose2d hubPose2d = Field.getAllianceHubPose();
    public static enum Targets {
        ZONE1 (FieldConstants.ZONE1_RADIUS.to(PARTsUnitType.Meter)),
        ZONE2 (FieldConstants.ZONE2_RADIUS.to(PARTsUnitType.Meter)),
        ZONE3 (FieldConstants.ZONE3_RADIUS.to(PARTsUnitType.Meter)),
        ZONE4 (FieldConstants.ZONE4_RADIUS.to(PARTsUnitType.Meter));

        private double radius;

        Targets (double radius) {
            this.radius = radius;
        }

        public double getRadius () {
            return radius;
        }
    }

    public static boolean isInRadius (Pose2d center, Pose2d point, double radius) {
        double centerPoseX = center.getX();
        double centerPoseY = center.getY();

        double pointPoseX = point.getX();
        double pointPoseY = point.getY();

        double distanceSquared = (Math.pow(centerPoseX - pointPoseX, 2) + Math.pow(centerPoseY - pointPoseY, 2));
        //System.out.println(Math.sqrt(distanceSquared));
        return distanceSquared <= Math.pow(radius, 2);
    }

    public static Targets getZone (Pose2d point) {
        if (isInRadius(hubPose2d, point, Targets.ZONE1.getRadius())) {
            return Targets.ZONE1;
        }

        else if (isInRadius(hubPose2d, point, Targets.ZONE2.getRadius())) {
            return Targets.ZONE2;
        }

        else if (isInRadius(hubPose2d, point, Targets.ZONE3.getRadius())) {
            return Targets.ZONE3;
        }

        else if (isInRadius(hubPose2d, point, Targets.ZONE4.getRadius())) {
            return Targets.ZONE4;
        }

        else {
            return null;
        }
    }
}
