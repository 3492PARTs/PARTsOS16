package frc.robot.constants;

public class ClimberConstants {
    public enum ClimberState {
        IDLE(0),
        DISABLED(0),
        CLIMBING(0.1),
        DECLIMB(-0.1);

        private final double speed;

        ClimberState(double speed) {
            this.speed = speed;
        }

        public double getSpeed() {
            return speed;
        }
    }

    public static final int CLIMBER_MOTOR_ID = 32;

    public static final double CLIMBER_GEAR_RATIO = 4.0/1.0;
}
