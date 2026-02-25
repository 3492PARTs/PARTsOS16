package frc.robot.util;

import java.util.Optional;

import org.parts3492.partslib.PARTsUnit;
import org.parts3492.partslib.PARTsUnit.PARTsUnitType;
import org.parts3492.partslib.network.PARTsNT;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;

public class Hub {
    private static Pose2d hubPose2d = Field.getAllianceHubPose();
    private static Timer timer = new Timer();
    private static boolean previousHubActive = true;
    private static PARTsNT partsNT = new PARTsNT("Hub");

    public static enum Targets {
        DEADZONE(new PARTsUnit(8, PARTsUnitType.Foot).to(PARTsUnitType.Meter)),
        ZONE1(new PARTsUnit(10, PARTsUnitType.Foot).to(PARTsUnitType.Meter)),
        ZONE2(new PARTsUnit(12, PARTsUnitType.Foot).to(PARTsUnitType.Meter)),
        ZONE3(new PARTsUnit(14, PARTsUnitType.Foot).to(PARTsUnitType.Meter)),
        ZONE4(new PARTsUnit(16, PARTsUnitType.Foot).to(PARTsUnitType.Meter));

        private double radius;

        Targets(double radius) {
            this.radius = radius;
        }

        public double getRadius() {
            return radius;
        }
    }

    public static void outputTelemetry() {
        partsNT.putBoolean("Hub Active", Hub.isHubActive());
        partsNT.putDouble("Time Left", timer.get() <=25 ? 25 - timer.get() : 0);
        checkHubActivity();
    }

    public static boolean isInRadius(Pose2d center, Pose2d point, double radius) {
        double centerPoseX = center.getX();
        double centerPoseY = center.getY();

        double pointPoseX = point.getX();
        double pointPoseY = point.getY();

        double distanceSquared = (Math.pow(centerPoseX - pointPoseX, 2) + Math.pow(centerPoseY - pointPoseY, 2));
        return distanceSquared <= Math.pow(radius, 2);
    }

    public static Targets getZone(Pose2d point) {
        if (isInRadius(hubPose2d, point, Targets.DEADZONE.getRadius())) {
            return null;
        }
        else if (isInRadius(hubPose2d, point, Targets.ZONE1.getRadius())) {
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

    public static boolean isHubActive() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        // If we have no alliance, we cannot be enabled, therefore no hub.
        if (alliance.isEmpty()) {
            return false;
        }
        // Hub is always enabled in autonomous.
        if (DriverStation.isAutonomousEnabled()) {
            return true;
        }
        // At this point, if we're not teleop enabled, there is no hub.
        if (!DriverStation.isTeleopEnabled()) {
            return false;
        }

        // We're teleop enabled, compute.
        double matchTime = DriverStation.getMatchTime();
        String gameData = DriverStation.getGameSpecificMessage();
        // If we have no game data, we cannot compute, assume hub is active, as its
        // likely early in teleop.
        if (gameData.isEmpty()) {
            return true;
        }
        boolean redInactiveFirst = false;
        switch (gameData.charAt(0)) {
            case 'R' -> redInactiveFirst = true;
            case 'B' -> redInactiveFirst = false;
            default -> {
                // If we have invalid game data, assume hub is active.
                return true;
            }
        }

        // Shift was is active for blue if red won auto, or red if blue won auto.
        boolean shift1Active = switch (alliance.get()) {
            case Red -> !redInactiveFirst;
            case Blue -> redInactiveFirst;
        };

        if (matchTime > 130) {
            // Transition shift, hub is active.
            return true;
        } else if (matchTime > 105) {
            // Shift 1
            return shift1Active;
        } else if (matchTime > 80) {
            // Shift 2
            return !shift1Active;
        } else if (matchTime > 55) {
            // Shift 3
            return shift1Active;
        } else if (matchTime > 30) {
            // Shift 4
            return !shift1Active;
        } else {
            // End game, hub always active.
            return true;
        }
    }

    public static void startHubActiveTimer() {
        timer.start();
    }

    public static void checkHubActivity() {
        if (previousHubActive != Hub.isHubActive()) {
            timer.restart();
        }
        previousHubActive = Hub.isHubActive();
    }
}
