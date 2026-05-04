package frc.robot.constants;

import org.parts3492.partslib.PARTsUnit;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;

//move to constants pls

/** This interface stores information about each camera. */
public interface CameraConstants {
    public enum CameraName {
        LEFT_FRONT_CAMERA("limelight-johnny"),
        LEFT_SIDE_CAMERA("limelight-sue"),
        RIGHT_FRONT_CAMERA("limelight-silver"),
        RIGHT_SIDE_CAMERA("limelight-surfer");

        private String cameraName = "";

        private CameraName(String name) {
            cameraName = name;
        }

        public String getCameraName() {
            return cameraName;
        }
    }

    public enum Pipelines {
        MAIN(1),
        VIEWING(1);

        private int pipelineIndex;

        private Pipelines(int index) {
            pipelineIndex = index;
        }

        public int getIndex() {
            return pipelineIndex;
        }
    }

    public Camera[] LimelightCameras = new Camera[] {
            new Camera(CameraName.LEFT_FRONT_CAMERA.getCameraName(),
                    new Pose3d(PARTsUnit.InchesToMeters.apply(13.5), PARTsUnit.InchesToMeters.apply(9.75),
                            PARTsUnit.InchesToMeters.apply(13.0),
                            new Rotation3d(0, 0,
                                    0)),
                    true),
            new Camera(CameraName.LEFT_SIDE_CAMERA.getCameraName(),
                    new Pose3d(PARTsUnit.InchesToMeters.apply(9.75), PARTsUnit.InchesToMeters.apply(12.25),
                            PARTsUnit.InchesToMeters.apply(13.0),
                            new Rotation3d(0, 0,
                                    Units.degreesToRadians(90))),
                    true),
            new Camera(CameraName.RIGHT_FRONT_CAMERA.getCameraName(),
                    new Pose3d(PARTsUnit.InchesToMeters.apply(13.5), -PARTsUnit.InchesToMeters.apply(9.75),
                            PARTsUnit.InchesToMeters.apply(13.0),
                            new Rotation3d(0, 0,
                                    0)),
                    true),
            new Camera(CameraName.RIGHT_SIDE_CAMERA.getCameraName(),
                    new Pose3d(PARTsUnit.InchesToMeters.apply(9.0), -PARTsUnit.InchesToMeters.apply(11.75),
                            PARTsUnit.InchesToMeters.apply(13.0),
                            new Rotation3d(0, 0,
                                    Units.degreesToRadians(-90))),
                    true)
    };

    public static class Camera {
        private String name;
        private Pose3d location;
        private Boolean isEnabled;

        public Camera(String name, Pose3d location, Boolean isEnabled) {
            this.name = name;
            this.location = location;
            this.isEnabled = isEnabled;
        }

        public String getName() {
            return name;
        }

        public Pose3d getLocation() {
            return location;
        }

        public boolean isEnabled() {
            return isEnabled;
        }

        public void setEnabled(boolean enabled) {
            this.isEnabled = enabled;
        }
    }
}