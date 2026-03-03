package frc.robot.constants;

public class KickerConstants {
    public enum KickerState {
        IDLE(0),
        DISABLED(0),
        ROLLING(1);

        private final double speed;

        KickerState(double speed) {
            this.speed = speed;
        }

        public double getSpeed() {
            return speed;
        }
    }

    public static final int KICKER_MOTOR_ID = 34;
    public static final String CAN_BUS_NAME = "bye";
}
