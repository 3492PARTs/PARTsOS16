package frc.robot.subsystems;

import java.util.function.BiFunction;
import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.constants.CameraConstants;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.CameraConstants.Camera;
import frc.robot.constants.CameraConstants.Pipelines;
import frc.robot.constants.VisionConstants;
import frc.robot.util.Field;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.Trench;
import frc.robot.util.LimelightHelpers.PoseEstimate;

import org.parts3492.partslib.PARTsUnit;
import org.parts3492.partslib.PARTsUnit.PARTsUnitType;
import org.parts3492.partslib.command.PARTsCommandUtils;
import org.parts3492.partslib.command.PARTsSubsystem;

public class LimelightVision extends PARTsSubsystem {

    private final Supplier<Pose2d> poseSupplier;
    private final BiFunction<Pose2d, Double, Boolean> addVisionMeasurementBiFunction;
    private final Consumer<Vector<N3>> setVisionMeasurementStdDevsConsumer;

    public enum MegaTagMode {
        MEGATAG1,
        MEGATAG2
    }

    public enum WhitelistMode {
        BLUE_HUB_TAGS(Field.BLUE_HUB_TAG_IDS),
        RED_HUB_TAGS(Field.RED_HUB_TAG_IDS),
        ALL(Field.getAllTagIDs()),
        NONE(new int[0]);

        private int[] ids;

        private WhitelistMode(int... ids) {
            this.ids = ids;
        }

        public int[] getIds() {
            return this.ids;
        }
    }

    private MegaTagMode megaTagMode;
    private WhitelistMode whitelistMode;
    private int imuMode;
    private int maxTagCount;

    public LimelightVision(Supplier<Pose2d> poseSupplier,
            BiFunction<Pose2d, Double, Boolean> addVisionMeasurementBiFunction,
            Consumer<Vector<N3>> setVisionMeasurementStdDevsConsumer, Consumer<Pose2d> resetPoseConsumer) {
        super("LimelightVision");
        this.poseSupplier = poseSupplier;
        this.addVisionMeasurementBiFunction = addVisionMeasurementBiFunction;
        this.setVisionMeasurementStdDevsConsumer = setVisionMeasurementStdDevsConsumer;

        for (Camera camera : CameraConstants.LimelightCameras) {
            Pose3d robotRelativePose = camera.getLocation();
            LimelightHelpers.setCameraPose_RobotSpace(
                    camera.getName(),
                    robotRelativePose.getX(),
                    -robotRelativePose.getY(),
                    robotRelativePose.getZ(),
                    Units.radiansToDegrees(robotRelativePose.getRotation().getX()),
                    Units.radiansToDegrees(robotRelativePose.getRotation().getY()),
                    Units.radiansToDegrees(robotRelativePose.getRotation().getZ()));
        }

        maxTagCount = 0;

        setMegaTagMode(MegaTagMode.MEGATAG2);
        setWhitelistMode(WhitelistMode.ALL);
        setIMUMode(1);

        super.partsNT.putSmartDashboardSendable("Set MT-1", commandMegaTagMode(MegaTagMode.MEGATAG1),
                !RobotConstants.COMPETITION);
        super.partsNT.putSmartDashboardSendable("Set MT-2", commandMegaTagMode(MegaTagMode.MEGATAG2),
                !RobotConstants.COMPETITION);
    }

    public void setMegaTagMode(MegaTagMode mode) {
        this.megaTagMode = mode;
        switch (mode) {
            case MEGATAG1:
                setVisionMeasurementStdDevsConsumer.accept(VisionConstants.MT1_STDEVS);
                break;
            case MEGATAG2:
                setVisionMeasurementStdDevsConsumer.accept(VisionConstants.MT2_STDEVS);
                break;
        }
        partsNT.putString("Megatag Mode", getMTmode().toString(), true);
    }

    public Command commandMegaTagMode(MegaTagMode mode) {
        Command c = PARTsCommandUtils.setCommandName("LimelightVision.commandMegaTagMode",
                this.runOnce(() -> setMegaTagMode(mode)));
        c = c.ignoringDisable(true);
        return c;
    }

    public void setWhitelistMode(WhitelistMode mode) {
        this.whitelistMode = mode;
        switch (mode) {
            case BLUE_HUB_TAGS:
                setTagWhitelist(WhitelistMode.BLUE_HUB_TAGS.getIds());
                break;
            case RED_HUB_TAGS:
                setTagWhitelist(WhitelistMode.RED_HUB_TAGS.getIds());
                break;
            case ALL:
                setTagWhitelist(WhitelistMode.ALL.getIds());
                break;
            case NONE:
                setTagWhitelist(WhitelistMode.NONE.getIds());
                break;
        }
    }

    public WhitelistMode getWhitelistMode() {
        return this.whitelistMode;
    }

    private void setTagWhitelist(int... ids) {
        for (Camera camera : CameraConstants.LimelightCameras) {
            LimelightHelpers.SetFiducialIDFiltersOverride(camera.getName(), ids);
        }
    }

    public void setIMUMode(int mode) {
        this.imuMode = mode;
        for (Camera camera : CameraConstants.LimelightCameras) {
            LimelightHelpers.SetIMUMode(camera.getName(), mode);
        }
    }

    public int getMaxTagCount() {
        return this.maxTagCount;
    }

    public MegaTagMode getMTmode() {
        return megaTagMode;
    }

    public PoseEstimate getMegaTag1PoseEstimate(String limelightName) {
        return LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
    }

    private PoseEstimate getMegaTag2PoseEstimate(String limelightName) {
        return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
    }

    private boolean robotIsOnBlueSide() {
        Pose2d pose = poseSupplier.get();
        return pose.getX() < Field.LENGTH / 2 == RobotContainer.isBlue();
    }

    private void updateWhitelistMode() {
        if (robotIsOnBlueSide() && getWhitelistMode() == WhitelistMode.RED_HUB_TAGS) {
            setWhitelistMode(WhitelistMode.BLUE_HUB_TAGS);
        }
        if (!robotIsOnBlueSide() && getWhitelistMode() == WhitelistMode.BLUE_HUB_TAGS) {
            setWhitelistMode(WhitelistMode.RED_HUB_TAGS);
        }
    }

    public static boolean cameraSeesTag(String cameraName) {
        return LimelightHelpers.getTV(cameraName);
    }

    public static int getVisibleTagId(String cameraName) {
        try {
            double[] targetArray = LimelightHelpers.getT2DArray(cameraName);
            return (int) targetArray[9];
        } catch (ArrayIndexOutOfBoundsException a) {
            return -1;
        }
    }

    @Override
    public void periodic() {
        this.maxTagCount = 0;

        updateWhitelistMode();
        for (Camera camera : CameraConstants.LimelightCameras) {
            int tagId = -1;
            PoseEstimate poseEstimate = null;
            boolean inRadius = false, data = false, accepted = false;

            double[] hw = LimelightHelpers.getLimelightDoubleArrayEntry("limelight", "hw").get();
            partsNT.putDouble(camera.getName() + "/temp", hw.length > 0 ? hw[0] : -1, !RobotConstants.COMPETITION); // loop-overrun

            if (camera.isEnabled()) {
                LimelightHelpers.SetRobotOrientation(
                        camera.getName(),
                        // i think this is still needed b/c if we always assume blue on red we start
                        // backwards.
                        (poseSupplier.get().getRotation().getDegrees()) % 360,
                        // we may need to consider these values for when we go ove the bump
                        // if we are at an angle on the bump it could throw our esimates off
                        0,
                        0,
                        0,
                        0,
                        0);

                tagId = (int) getVisibleTagId(camera.getName());

                poseEstimate = (megaTagMode == MegaTagMode.MEGATAG2)
                        ? getMegaTag2PoseEstimate(camera.getName())
                        : getMegaTag1PoseEstimate(camera.getName());

                inRadius = tagId != -1 && Field.isInRadius(Field.getTag(tagId).getLocation().toPose2d(),
                        poseEstimate.pose, new PARTsUnit(12, PARTsUnitType.Foot).to(PARTsUnitType.Meter));
                partsNT.putDouble(camera.getName() + "/Distance",
                        tagId != -1
                                ? Trench.getDistance(poseEstimate.pose, Field.getTag(tagId).getLocation().toPose2d())
                                : -1,
                        !RobotConstants.COMPETITION);
                int requiredTagCount = (megaTagMode == MegaTagMode.MEGATAG1) ? 2 : 1;

                if (poseEstimate != null && poseEstimate.tagCount >= requiredTagCount && inRadius) {
                    data = true;
                    accepted = addVisionMeasurementBiFunction.apply(poseEstimate.pose,
                            poseEstimate.timestampSeconds);

                    maxTagCount = Math.max(maxTagCount, poseEstimate.tagCount);
                }

                partsNT.putBoolean(camera.getName() + "/Has Data", data, !RobotConstants.COMPETITION); // loop-overrun
                partsNT.putBoolean(camera.getName() + "/Accepted Data", accepted, !RobotConstants.COMPETITION); // loop-overrun

                partsNT.putNumber(camera.getName() + "/X", poseEstimate == null ? -1 : poseEstimate.pose.getX(),
                        !RobotConstants.COMPETITION); // loop-overrun
                partsNT.putNumber(camera.getName() + "/Y", poseEstimate == null ? -1 : poseEstimate.pose.getY(),
                        !RobotConstants.COMPETITION); // loop-overrun
                partsNT.putNumber(camera.getName() + "/Rotation (deg)",
                        poseEstimate == null ? -1 : poseEstimate.pose.getRotation().getDegrees(),
                        !RobotConstants.COMPETITION); // loop-overrun

                partsNT.putNumber(camera.getName() + "/tag id", tagId, !RobotConstants.COMPETITION);
                partsNT.putNumber(camera.getName() + "/tag count", poseEstimate == null ? -1 : poseEstimate.tagCount,
                        !RobotConstants.COMPETITION);
                partsNT.putBoolean(camera.getName() + "/In Radius", inRadius, !RobotConstants.COMPETITION);
            }
        }
    }

    @Override
    public void outputTelemetry() {
        partsNT.putString("Whitelist Mode", getWhitelistMode().toString(), RobotContainer.debug);
        partsNT.putNumber("IMU Mode", imuMode, RobotContainer.debug);
    }

    @Override
    public void stop() {
        setPipelineIndex(Pipelines.VIEWING);
    }

    @Override
    public void reset() {
    }

    @Override
    public void log() {
        // TODO Auto-generated method stub
        // throw new UnsupportedOperationException("Unimplemented method 'log'");
    }

    public void setPipelineIndex(Pipelines pipeline) {
        partsNT.putString("Pipeline name", pipeline.name(), true);
        for (Camera camera : CameraConstants.LimelightCameras) {
            if (camera.isEnabled()) {
                LimelightHelpers.setLEDMode_PipelineControl(camera.getName());
                LimelightHelpers.setPipelineIndex(camera.getName(), pipeline.getIndex());
            }
        }
    }
}