package frc.robot.states;

/** The state that the Kicker is in. */
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
