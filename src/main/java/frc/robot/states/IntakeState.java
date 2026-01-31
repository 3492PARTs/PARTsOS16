package frc.robot.states;

public enum IntakeState {
    IDLE(0),
    DISABLED(0),
    INTAKING(0),
    OUTTAKING(0);
    private double speed;

    private IntakeState(double speed){
        this.speed = speed;
    }

    public double getSpeed(){
        return speed;
    }
}

