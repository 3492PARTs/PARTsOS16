package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import java.util.Optional;
import org.parts3492.partslib.network.PARTsNT;

public class Hub {
  private static Timer timer = new Timer();
  private static boolean previousHubActive = true;
  private static PARTsNT partsNT = new PARTsNT("Hub");

  public static enum Targets {
    BEHIND_HUB(),
    TRENCH();

    /*public void setFieldObject2d() {
        FieldObject2d targetFieldObject2d = Field.FIELD2D.getObject(this.name());
        Pose2d pose = new Pose2d(hubPose2d.get().getX() - this.radius * (RobotUtils.isBlue() ? 1 : -1),
                hubPose2d.get().getY(), hubPose2d.get().getRotation());
        targetFieldObject2d.setPose(pose);

        Pose2d poseRot = pose.rotateAround(hubPose2d.get().getTranslation(),
                new Rotation2d(new PARTsUnit(45, PARTsUnitType.Angle).to(PARTsUnitType.Radian)));

        FieldObject2d targetFieldObject2dRotated = Field.FIELD2D.getObject(this.name() + "Rot");
        targetFieldObject2dRotated.setPose(poseRot);

        Pose2d poseRotInv = pose.rotateAround(hubPose2d.get().getTranslation(),
                new Rotation2d(new PARTsUnit(-45, PARTsUnitType.Angle).to(PARTsUnitType.Radian)));
        FieldObject2d targetFieldObject2dRotatedInv = Field.FIELD2D.getObject(this.name() + "RotInv");
        targetFieldObject2dRotatedInv.setPose(poseRotInv);
    }*/
  }

  public static void outputTelemetry() {
    partsNT.putBoolean("Hub Active", Hub.isHubActive(), true);
    partsNT.putDouble("Time Left", timer.get() <= 25 ? 25 - timer.get() : 0, true);
    checkHubActivity();
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
    boolean shift1Active =
        switch (alliance.get()) {
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
