package frc.robot.constants;

public class HopperConstants {
    public enum HopperState {
        IDLE(0),
        DISABLED(0),
        ROLLING(0.75),
        BACKROLLING(-.75);

        private final double speed;

        HopperState(double speed) {
            this.speed = speed;
        }

        public double getSpeed() {
            return speed;
        }
    }

    public static final int HOPPER_MOTOR_ID = 37;
    public static final String CAN_BUS_NAME = "bye";
}
