package frc.robot.subsystems.Shooter;

public class ShooterSim extends Shooter {

    @Override
    protected void setSpeed(double speed) {
        partsNT.putDouble("Speed", speed);
    }

    @Override
    protected void setVoltage(double voltage) {
        partsNT.putDouble("Voltage", voltage);
    }

    @Override
    protected double getRPM() {
        return 0;
    }

    @Override
    protected double getVoltage() {
        return 0;
    }
    
}
