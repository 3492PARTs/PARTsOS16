package frc.robot.states;

/** The state the Hopper is in */
public enum HopperState {
    IDLE(0),
    DISABLED(0),
    ROLLING(1),
    BACKROLLING(-1);
    
    private final double speed;

    HopperState(double speed) {
        this.speed = speed;
    }

    public double getSpeed () {
        return speed;
    }
}
