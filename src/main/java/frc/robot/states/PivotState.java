package frc.robot.states;

/** The state that the Pivot is in. */
public enum PivotState {
    HOME(0),
    INTAKE(0);

    private double position;

    private PivotState(double position){
        this.position = position;
    }

    public double getPosition(){
        return position;
    }
}
