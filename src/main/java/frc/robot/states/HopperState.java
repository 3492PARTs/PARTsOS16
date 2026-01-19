package frc.robot.states;

import frc.robot.constants.HopperConstants;

/* The state the hopepr is in */
public enum HopperState {
    IDLE(0),
    DISABLED(0),
    EXTENDING(HopperConstants.EXTEND_SPEED),
    RETRACTING(HopperConstants.WITHDRAW_SPEED);
    
    private final double speed;

    HopperState(double speed) {
        this.speed = speed;
    }

    public double getSpeed () {
        return speed;
    }
}
