package frc.robot.states;

/** The state the Intake is in */
public enum IntakeState {
    IDLE(0, 180),
    DISABLED(0, 0),
    INTAKING(0.75, 190),
    OUTTAKING(-0.75, 190),
    SHOOTING(0.25, 90),
    HOME(0, 0);
    private double speed;
    private double angle;

    private IntakeState(double speed, double angle){
        this.speed = speed;
        this.angle = angle;
    }

    public double getSpeed(){
        return speed;
    }

    public double getAngle() {
        return angle;
    }
}

