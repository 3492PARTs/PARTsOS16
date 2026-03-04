package frc.robot.constants;

public class ClimberConstants {
    public enum ClimberState {
        IDLE(0),
        DISABLED(0),
        CLIMBING(ClimberConstants.CLIMBING_SPEED),
        DECLIMB(ClimberConstants.DECLIMBING_SPEED);

        private final double speed;

        ClimberState(double speed) {
            this.speed = speed;
        }

        public double getSpeed() {
            return speed;
        }
    }

    public static final double CLIMBING_SPEED = .1;
    public static final double DECLIMBING_SPEED = -.1;
    public static final int CLIMBER_MOTOR_ID = 0;

    public static final double CLIMBER_GEAR_RATIO = 4.0/1.0;
}
