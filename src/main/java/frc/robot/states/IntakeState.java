package frc.robot.states;

/** The state the Intake is in */
public enum IntakeState {
    IDLE(0, 0),
    DISABLED(0, 0),
    INTAKING(1, 185),
    OUTTAKING(-1, 185),
    SHOOTING(0.5, 90),
    TRAVELING(0, 160),
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

