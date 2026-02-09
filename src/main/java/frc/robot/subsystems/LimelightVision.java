package frc.robot.subsystems;

import java.util.function.BiConsumer;
import java.util.function.BiFunction;
import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.constants.CameraConstants;
import frc.robot.constants.CameraConstants.Camera;
import frc.robot.constants.CameraConstants.Pipelines;
import frc.robot.constants.VisionConstants;
import frc.robot.util.Field;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.PoseEstimate;
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

    public enum IMUMode {
        EXTERNAL(1), // Seed internal IMU
        INTERNAL(3), // Use internal IMU + MT1
        BOTH(4); // Use internal IMU + external IMU

        private int value;

        private IMUMode(int value) {
            this.value = value;
        }

        public int getValue() {
            return value;
        }
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
    private IMUMode imuMode;
    private int maxTagCount;

    public LimelightVision(Supplier<Pose2d> poseSupplier,
            BiFunction<Pose2d, Double, Boolean> addVisionMeasurementBiFunction,
            Consumer<Vector<N3>> setVisionMeasurementStdDevsConsumer) {
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

        setMegaTagMode(MegaTagMode.MEGATAG1);
        setWhitelistMode(WhitelistMode.ALL);
        setIMUMode(IMUMode.EXTERNAL);

        super.partsNT.putSmartDashboardSendable("Set MT-1", commandMegaTagMode(MegaTagMode.MEGATAG1));
        super.partsNT.putSmartDashboardSendable("Set MT-2", commandMegaTagMode(MegaTagMode.MEGATAG2));
        super.partsNT.putSmartDashboardSendable("Set EXTERNAL IMU", commandIMUMode(IMUMode.EXTERNAL));
        super.partsNT.putSmartDashboardSendable("Set INTERNAL IMU", commandIMUMode(IMUMode.INTERNAL));
        super.partsNT.putSmartDashboardSendable("Set BOTH IMU", commandIMUMode(IMUMode.BOTH));
        super.partsNT.putSmartDashboardSendable("Set VIEWING PIPELINE", commandPipeline(Pipelines.VIEWING));
        super.partsNT.putSmartDashboardSendable("Set MAIN PIPELINE", commandPipeline(Pipelines.MAIN));
    }

    public void setMegaTagMode(MegaTagMode mode) {
        this.megaTagMode = mode;
        setVisionMeasurementStdDevs();
    }

    public void setVisionMeasurementStdDevs() {
        switch (this.megaTagMode) {
            case MEGATAG1:
                setVisionMeasurementStdDevsConsumer.accept(VisionConstants.MT1_STDEVS);
                break;
            case MEGATAG2:
                setVisionMeasurementStdDevsConsumer.accept(VisionConstants.MT2_STDEVS);
                break;
        }
    }

    public Command commandMegaTagMode(MegaTagMode mode) {
        Command c = PARTsCommandUtils.setCommandName("commandMegaTagMode", this.runOnce(() -> setMegaTagMode(mode)));
        c = c.ignoringDisable(true);
        return c;
    }

    public Command commandIMUMode(IMUMode mode) {
        Command c = PARTsCommandUtils.setCommandName("commandIMUMode", this.runOnce(() -> setIMUMode(mode)));
        c = c.ignoringDisable(true);
        return c;
    }

    public Command commandPipeline(Pipelines mode) {
        Command c = PARTsCommandUtils.setCommandName("commandPipeline", this.runOnce(() -> setPipelineIndex(mode)));
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

    public void setIMUMode(IMUMode mode) {
        this.imuMode = mode;
        for (Camera camera : CameraConstants.LimelightCameras) {
            LimelightHelpers.SetIMUMode(camera.getName(), mode.getValue());
        }
    }

    public int getMaxTagCount() {
        return this.maxTagCount;
    }

    public MegaTagMode getMTmode() {
        return megaTagMode;
    }

    public IMUMode getIMUmode() {
        return imuMode;
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
        partsNT.putNumber("Robot Rotation (deg)",
                (poseSupplier.get().getRotation().getDegrees()) % 360);

        for (Camera camera : CameraConstants.LimelightCameras) {
            boolean hasData = false;
            int tagCount = 0;
            boolean measurementAccepted = false;
            // this only works if the robots pose is set correctly with MT1
            LimelightHelpers.SetRobotOrientation(
                    camera.getName(),
                    (poseSupplier.get().getRotation().getDegrees()) % 360,
                    0,
                    0,
                    0,
                    0,
                    0);
            if (camera.isEnabled()) {
                PoseEstimate poseEstimate = (megaTagMode == MegaTagMode.MEGATAG2)
                        ? getMegaTag2PoseEstimate(camera.getName())
                        : getMegaTag1PoseEstimate(camera.getName());

                partsNT.putNumber(camera.getName() + "/X", poseEstimate.pose.getX());
                partsNT.putNumber(camera.getName() + "/Y", poseEstimate.pose.getY());
                partsNT.putNumber(camera.getName() + "/Rotation (deg)", poseEstimate.pose.getRotation().getDegrees());

                if (poseEstimate != null && poseEstimate.tagCount > 0) {

                    hasData = true;
                    tagCount = poseEstimate.tagCount;

                    maxTagCount = Math.max(maxTagCount, poseEstimate.tagCount);

                    boolean rejectUpdate = false;

                    if (poseEstimate.tagCount <= 1)
                        rejectUpdate = true;

                    if (this.megaTagMode == MegaTagMode.MEGATAG1) {
                        if (poseEstimate.rawFiducials.length > 0) {
                            if (poseEstimate.rawFiducials[0].ambiguity > .7)
                                rejectUpdate = true;
                            if (poseEstimate.rawFiducials[0].distToCamera > 3)
                                rejectUpdate = true;
                        } else
                            rejectUpdate = true;
                    }

                    if (!rejectUpdate) {
                        setVisionMeasurementStdDevs();
                        Boolean result = addVisionMeasurementBiFunction.apply(poseEstimate.pose,
                                poseEstimate.timestampSeconds);
                        measurementAccepted = result;
                    } else {
                        measurementAccepted = false;
                    }
                } else {
                    hasData = false;
                    tagCount = 0;
                    measurementAccepted = false;
                }
                partsNT.putBoolean(camera.getName() + "/Has Data", hasData);
                partsNT.putNumber(camera.getName() + "/Tag Count", tagCount);
                partsNT.putBoolean(camera.getName() + "/Measurement Accepted", measurementAccepted);
            }
        }
    }

    @Override
    public void outputTelemetry() {
        partsNT.putString("Megatag Mode", getMTmode().toString());
        partsNT.putString("Whitelist Mode", getWhitelistMode().toString());
        partsNT.putString("IMU Mode", getIMUmode().toString());
    }

    @Override
    public void stop() {
        setPipelineIndex(Pipelines.VIEWING);
        setIMUMode(IMUMode.EXTERNAL);
    }

    @Override
    public void reset() {
        // TODO Auto-generated method stub
        // throw new UnsupportedOperationException("Unimplemented method 'reset'");
    }

    @Override
    public void log() {
        // TODO Auto-generated method stub
        // throw new UnsupportedOperationException("Unimplemented method 'log'");
    }

    public void setPipelineIndex(Pipelines pipeline) {
        partsNT.putString("Pipeline name", pipeline.name());
        for (Camera camera : CameraConstants.LimelightCameras) {
            if (camera.isEnabled()) {
                LimelightHelpers.setLEDMode_PipelineControl(camera.getName());
                LimelightHelpers.setPipelineIndex(camera.getName(), pipeline.getIndex());
            }
        }
    }
}