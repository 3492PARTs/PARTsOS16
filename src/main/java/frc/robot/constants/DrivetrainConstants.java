package frc.robot.constants;

import org.parts3492.partslib.PARTsUnit;
import org.parts3492.partslib.PARTsUnit.PARTsUnitType;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N3;

public class DrivetrainConstants {
        public static final String CAN_BUS_NAME = "hi";
        public static final double MAX_ANGULAR_SPEED = Math.PI / 4; // Radians
        public static final double SPEED_PERCENT = 1; // percentage (0 to 1)

        public static final double MAX_AIM_VELOCITY = 75 * Math.PI; // radd/s 50
        public static final double MAX_AIM_ACCELERATION = 20 * Math.PI; // rad/s^2 10
        public static final double MAX_RANGE_VELOCITY = 2;// m/s
        public static final double MAX_RANGE_ACCELERATION = 2;// m/2^s

        public static final double THETA_P = 7; // Proprotinal //4.5 15
        public static final double THETA_I = 0; // Gradual corretction
        public static final double THETA_D = 0; // Smooth oscilattions

        public static final double RANGE_X_P = 16; // 6.5
        public static final double RANGE_I = 0;
        public static final double RANGE_D = 0;

        public static final double RANGE_Y_P = 4.5; // 4.5

        public static final PARTsUnit Y_RANGE_CONTROLLER_TOLERANCE = new PARTsUnit(1, PARTsUnitType.Inch);
        public static final PARTsUnit X_RANGE_CONTROLLER_TOLERANCE = new PARTsUnit(2, PARTsUnitType.Inch);
        public static final PARTsUnit THETA_CONTROLLER_TOLERANCE = new PARTsUnit(1, PARTsUnitType.Angle);

        public static final double ALIGN_TIMEOUT = 0.3; // seconds

        /*
         * “During each predict/update cycle,
         * assume my drivetrain model could be wrong by about
         * ±0.25 m in X, ±0.25 m in Y, and ±0.20 rad (~±11.5°) in heading (1σ scale).”
         * 
         * If you make the std devs bigger
         * The estimator thinks: “My odometry prediction is shaky.”
         * Result:
         * It will trust odometry less
         * It will trust measurements (like vision) more relative to odometry
         * Your estimate will tend to snap/shift more toward vision when vision updates
         * arrive
         * Drift from bad wheel slip is less “baked in”
         * This is why we increase them on a hump: wheel slip/bounce makes the
         * prediction less reliable.
         * 
         * If you make the std devs smaller
         * The estimator thinks: “My odometry prediction is solid.”
         * Result:
         * It will stick with odometry more
         * It will resist vision corrections more (vision has to “argue harder” to move
         * the pose)
         * Pose will be smoother, but it can be consistently wrong if odometry is wrong
         * (like on the hump)
         */

        // ---- Tune these numbers ----
        // "Normal" (flat) drivetrain model uncertainty (std dev).
        // Smaller = we trust the drivetrain model/odometry more.
        private static final double kFlatXStdDevMeters = 0.03;
        private static final double kFlatYStdDevMeters = 0.03;
        private static final double kFlatThetaStdDevRad = 0.03; // ~1.7 deg

        // What setStateStdDevs means:
        // - These are the assumed standard deviations of the *state prediction step*.
        // - Bigger values => "my predicted motion is uncertain"
        // - That makes the filter rely relatively more on measurement updates (e.g.,
        // vision).
        public static final edu.wpi.first.math.Vector<N3> STABLE_STDEVS = VecBuilder.fill(kFlatXStdDevMeters,
                        kFlatYStdDevMeters, kFlatThetaStdDevRad);

        // "Hump" uncertainty: larger = we trust drivetrain odometry less.
        // Start here, then tune.
        private static final double kHumpXStdDevMeters = 0.25;
        private static final double kHumpYStdDevMeters = 0.25;
        private static final double kHumpThetaStdDevRad = 0.20; // ~11.5 deg

        // Apply std devs for hump driving (slip/bounce expected).
        public static final edu.wpi.first.math.Vector<N3> AIRBORNE_STDEVS = VecBuilder.fill(kHumpXStdDevMeters,
                        kHumpYStdDevMeters, kHumpThetaStdDevRad);
}
