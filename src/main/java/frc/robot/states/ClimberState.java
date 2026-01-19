package frc.robot.states;

import frc.robot.constants.ClimberConstants;

/*States the Climber can be in */
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
