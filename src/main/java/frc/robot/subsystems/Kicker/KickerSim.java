package frc.robot.subsystems.Kicker;

public class KickerSim extends Kicker {
    @Override
    protected void setSpeed(double speed) {
        partsNT.putDouble("Our Kicker Speed", speed, true);
    }

    @Override
    protected double getRPM() {
        return -1;
    }

    @Override
    protected void setVoltage(double voltage) {
    }

    @Override
    protected double getVoltage() {
        return -1;
    }
}
