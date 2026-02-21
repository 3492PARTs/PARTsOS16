package frc.robot.states;

import frc.robot.constants.KickerConstants;

/** The state that the Kicker is in. */
public enum KickerState {
    IDLE(0),
    DISABLED(0),
    ROLLING(KickerConstants.KICKER_SPEED);

    private final double speed;

    KickerState(double speed) {
        this.speed = speed;
    }

    public double getSpeed() {
        return speed;
    }
}
