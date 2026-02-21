package frc.robot.states;

import frc.robot.constants.HopperConstants;

/** The state the Hopper is in */
public enum HopperState {
    IDLE(0),
    DISABLED(0),
    ROLLING(HopperConstants.ROLL_SPEED),
    BACKROLLING(HopperConstants.BACKROLL_SPEED);
    
    private final double speed;

    HopperState(double speed) {
        this.speed = speed;
    }

    public double getSpeed () {
        return speed;
    }
}
