package frc.robot.util;

import java.util.Optional;

import org.parts3492.partslib.PARTsUnit;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.LimelightVision.MegaTagMode;
import frc.robot.util.LimelightHelpers.PoseEstimate;
import frc.robot.util.LimelightHelpers.RawFiducial;

public class PARTsVisionFilter {
  private static final double kMaxLatencySec = 0.080;            // reject if older than 80ms
  private static final double kMaxAmbiguity = 0.20;              // single-tag ambiguity cutoff
  private static final double kMinTa = 0.08;                     // minimum target area (% of image)
  private static final double kMaxTagDistM_Single = 3.0;         // single-tag max distance
  private static final double kMaxTagDistM_Multi  = 6.0;         // multi-tag max distance
  private static final double kMaxXYJumpM = 0.75;                // teleport guard between updates
  private static final double kMaxHeadingJumpDeg = 30.0;         // yaw teleport guard
  private static final double kMaxRobotSpeedMpsForVision = 4.0;  // optional: ignore at high speed
  private static final double kMaxOmegaRadpsForVision = 8.0;     // optional: ignore at high omega

  // Field guards (update for your season)
  private static final double kFieldLengthM = 16.54;
  private static final double kFieldWidthM  = 8.21;
  private static final double kFieldBorderM = 0.50;

  public record PARTsVisionMeasurement(Pose2d pose, double timestampSec, Vector<N3> stdDevs, int tagId, int tagCount) {}

  /**
   * Produce a filtered vision measurement if Limelight data is trustworthy.
   *
   * @param limelightName Limelight NT name (ex: "limelight-front")
   * @param megatagMode MegaTag1 or MegaTag2
   * @param currentEstimatedPose your pose estimator's current estimated pose
   * @param robotSpeedMps current chassis linear speed magnitude
   * @param omegaRadps current chassis angular speed magnitude
   */
  public static Optional<PARTsVisionMeasurement> getTrustedMeasurement(
      String limelightName,
      MegaTagMode megatagMode,
      Pose2d currentEstimatedPose,
      PARTsUnit robotSpeedMps,
      PARTsUnit omegaRadps
  ) {
    // ---- 1) Basic "do we even have a target?" gate ----
    if (!LimelightHelpers.getTV(limelightName)) return Optional.empty();

    // ---- 2) Fetch pose estimate using LimelightHelpers ----
    PoseEstimate est = getPoseEstimate(limelightName, megatagMode);
    if (!LimelightHelpers.validPoseEstimate(est)) return Optional.empty();

    Pose2d visionPose = est.pose;

    // ---- 3) Freshness / latency gate ----
    double now = Timer.getFPGATimestamp();
    double age = now - est.timestampSeconds;
    if (age < 0.0 || age > kMaxLatencySec) return Optional.empty();

    // ---- 4) Motion gates (optional) ----
    if (robotSpeedMps.getValue() > kMaxRobotSpeedMpsForVision) return Optional.empty();
    if (omegaRadps.getValue() > kMaxOmegaRadpsForVision) return Optional.empty();

    // ---- 5) Confidence gates from tag metrics ----
    // Note: PoseEstimate.avgTagArea is already “% of image” per LimelightHelpers printPoseEstimate().
    if (est.avgTagArea < kMinTa) return Optional.empty();

    int tagCount = est.tagCount;
    double distM = est.avgTagDist;

    if (tagCount <= 0) return Optional.empty();

    if (tagCount == 1) {
      double ambiguity = getSingleTagAmbiguity(est);
      if (ambiguity > kMaxAmbiguity) return Optional.empty();
      if (distM > kMaxTagDistM_Single) return Optional.empty();
    } else {
      if (distM > kMaxTagDistM_Multi) return Optional.empty();
    }

    // ---- 6) Field boundary sanity ----
    if (!isReasonablyOnField(visionPose.getTranslation())) return Optional.empty();

    // ---- 7) Teleport guard vs current estimate ----
    if (!isPlausibleJump(currentEstimatedPose, visionPose)) return Optional.empty();

    // ---- 8) Dynamic standard deviations ----
    Vector<N3> stdDevs = computeStdDevs(tagCount, est.avgTagArea, distM);

    return Optional.of(new PARTsVisionMeasurement(visionPose, est.timestampSeconds, stdDevs, getVisibleTagId(limelightName), est.tagCount));
  }

  private static PoseEstimate getPoseEstimate(
      String limelightName,
      MegaTagMode megaTagMode
  ) {
    return switch (megaTagMode) {
      case MEGATAG2 -> LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);

      case MEGATAG1 -> LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
    };
  }

  /**
   * For single-tag solves, ambiguity is reported per-raw-fiducial.
   * If array is missing/mismatched, return a very large ambiguity to force rejection.
   */
  private static double getSingleTagAmbiguity(PoseEstimate est) {
    RawFiducial[] raws = est.rawFiducials;
    if (raws == null || raws.length != 1) return Double.POSITIVE_INFINITY;
    return raws[0].ambiguity;
  }

  private static boolean isReasonablyOnField(Translation2d t) {
    return t.getX() > -kFieldBorderM
        && t.getX() < kFieldLengthM + kFieldBorderM
        && t.getY() > -kFieldBorderM
        && t.getY() < kFieldWidthM + kFieldBorderM;
  }

  private static boolean isPlausibleJump(Pose2d current, Pose2d vision) {
    double dxy = current.getTranslation().getDistance(vision.getTranslation());
    double dheadingDeg = Math.abs(current.getRotation().minus(vision.getRotation()).getDegrees());
    return dxy <= kMaxXYJumpM && dheadingDeg <= kMaxHeadingJumpDeg;
  }

  private static Vector<N3> computeStdDevs(int tagCount, double ta, double distM) {
    // Base errors (meters, meters, radians)
    double xy = 0.60;
    double theta = Math.toRadians(20);

    // Improve with more tags
    double tagFactor = 1.0 / Math.min(tagCount, 4);

    // Improve with larger area (rough proxy for SNR)
    double areaFactor = clamp(0.5, 2.5, 0.25 / Math.max(ta, 1e-6));

    // Worse with distance (rough)
    double distFactor = clamp(0.7, 3.0, 0.7 + 0.25 * distM);

    double scale = tagFactor * areaFactor * distFactor;

    xy = clamp(0.05, 2.0, xy * scale);
    theta = clamp(Math.toRadians(2), Math.toRadians(45), theta * scale);

    return VecBuilder.fill(xy, xy, theta);
  }

  private static double clamp(double lo, double hi, double v) {
    return Math.max(lo, Math.min(hi, v));
  }

  public static int getVisibleTagId(String cameraName) {
        try {
            double[] targetArray = LimelightHelpers.getT2DArray(cameraName);
            return (int) targetArray[9];
        } catch (ArrayIndexOutOfBoundsException a) {
            return -1;
        }
    }
}