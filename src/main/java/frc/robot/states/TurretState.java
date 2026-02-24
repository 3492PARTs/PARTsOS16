package frc.robot.states;

/** The state that the Turret is in. */
public enum TurretState {
    DISABLED(0),
    IDLE(0),
    TRACKING(-1);

    private final double angle;

    TurretState(double angle){
        this.angle = angle;
    }

    public double getAngle() {
        return angle;
    }
}

