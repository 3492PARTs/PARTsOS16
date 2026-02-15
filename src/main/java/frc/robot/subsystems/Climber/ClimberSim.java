package frc.robot.subsystems.Climber;

public class ClimberSim extends Climber {
    @Override
    protected void setSpeed(double speed) {
        partsNT.putDouble("Our Climber Speed", speed);
    }
}
