package frc.robot.subsystems.Kicker;

public class KickerSim extends Kicker {
    @Override
    protected void setSpeed(double speed) {
        partsNT.putDouble("Our Kicker Speed", speed);
    }
}
