package frc.robot.states;

/** The state the Hopper is in */
public enum HopperState {
    IDLE(0),
    DISABLED(0),
    ROLLING(0.3),
    BACKROLLING(-0.3);
    
    private final double speed;

    HopperState(double speed) {
        this.speed = speed;
    }

    public double getSpeed () {
        return speed;
    }
}
