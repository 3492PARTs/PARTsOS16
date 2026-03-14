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
        LEFT_SIDE_CAMERA("limelight-frankie"),
        RIGHT_FRONT_ANGLE_CAMERA("limelight-ben");

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
                    new Pose3d(.343, .1905, .22225, // meters
                            new Rotation3d(0, Units.degreesToRadians(15), 0)),
                    true),
            new Camera(CameraName.LEFT_SIDE_CAMERA.getCameraName(),
                    new Pose3d(.22225, .3302, .22225, // meters
                            new Rotation3d(0, Units.degreesToRadians(15),
                                    Units.degreesToRadians(90))),
                    true),
            new Camera(CameraName.RIGHT_FRONT_ANGLE_CAMERA.getCameraName(),
                    new Pose3d(PARTsUnit.InchesToMeters.apply(13.0), -PARTsUnit.InchesToMeters.apply(10.0),
                            PARTsUnit.InchesToMeters.apply(14.0),
                            new Rotation3d(0, Units.degreesToRadians(40),
                                    -Units.degreesToRadians(45))),
                    false)
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