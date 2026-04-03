package frc.robot.subsystems.Hopper;

public class HopperSim extends Hopper{
    @Override
    protected void setSpeed(double speed) {
        partsNT.putDouble("Our Hopper Speed", speed, true);
    }

    @Override
    protected double getRPM() {
       return 0;
    }

    @Override
    protected void setVoltage(double voltage) {
        
    }

    @Override
    protected double getVoltage() {
       return 0;
    }
}

