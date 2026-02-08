package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;

//move to constants pls

/** This interface stores information about each camera. */
public interface CameraConstants {
    public enum CameraName {
        LEFT_FRONT_CAMERA("limelight-johnny", 4),
        LEFT_SIDE_CAMERA("limelight-frankie", 3),
        RIGHT_FRONT_CAMERA("limelight-ben",3),
        RIGHT_SIDE_CAMERA("limelight-sue", 4),
        FRONT_CENTER_CAMERA("limelight-reed", 4);

        private String cameraName = "";
        private int version;

        private CameraName(String name, int version) {
            cameraName = name;
            this.version = version;
        }

        public String getCameraName() {
            return cameraName;
        }

        public int getVersion() {
            return version;
        }
    }

    public enum Pipelines {
        MAIN(1),
        VIEWING (0);

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
                    true, CameraName.LEFT_FRONT_CAMERA.getVersion()),
            new Camera(CameraName.LEFT_SIDE_CAMERA.getCameraName(),
                    new Pose3d(.22225, .3302, .22225, // meters
                            new Rotation3d(0, Units.degreesToRadians(15),
                                    Units.degreesToRadians(90))),
                    true, CameraName.LEFT_SIDE_CAMERA.getVersion()),
            new Camera(CameraName.RIGHT_FRONT_CAMERA.getCameraName(),
                    new Pose3d(.343, -.1905, .22225, // meters
                            new Rotation3d(0, Units.degreesToRadians(15), 0)),
                    true, CameraName.RIGHT_FRONT_CAMERA.getVersion()),
            new Camera(CameraName.RIGHT_SIDE_CAMERA.getCameraName(),
                    new Pose3d(.22225, -.3302, .22225, // meters
                            new Rotation3d(0, Units.degreesToRadians(15),
                                    Units.degreesToRadians(-90))),        
                    true, CameraName.RIGHT_SIDE_CAMERA.getVersion()),
            new Camera(CameraName.FRONT_CENTER_CAMERA.getCameraName(),
                    new Pose3d(-.265, 0, .545, // meters
                            new Rotation3d(0, 0,
                                    Units.degreesToRadians(180))),
                    false, CameraName.FRONT_CENTER_CAMERA.getVersion())
    };

    public static class Camera {
        private String name;
        private Pose3d location;
        private Boolean isEnabled;
        private int version;

        public Camera(String name, Pose3d location, Boolean isEnabled, int version) {
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

        public int gerVersion() {
            return version;
        }
    }
}