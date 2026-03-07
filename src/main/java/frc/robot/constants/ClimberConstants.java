package frc.robot.constants;

public class ClimberConstants {
    public enum ClimberState {
        IDLE(0),
        DISABLED(0),
        CLIMBING(0.5),
        DECLIMB(-0.5);

        private final double speed;

        ClimberState(double speed) {
            this.speed = speed;
        }

        public double getSpeed() {
            return speed;
        }
    }

    public static final int CLIMBER_MOTOR_ID = 4;
    public static final String CAN_BUS_NAME = "hi";
    public static final double CLIMBER_GEAR_RATIO = 4.0/1.0;


}
