package frc.robot.states;

import frc.robot.constants.IntakeConstants;

/** The state the Intake is in */
public enum IntakeState {
    IDLE(0, 0),
    DISABLED(0, 0),
    INTAKING(IntakeConstants.INTAKE_SPEED, 90),
    OUTTAKING(-IntakeConstants.INTAKE_SPEED, 90),
    SHOOTING(0, 15);
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

