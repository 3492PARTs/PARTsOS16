package frc.robot.subsystems.Hopper;

public class HopperSim extends Hopper{
    @Override
    protected void setSpeed(double speed) {
        partsNT.putDouble("Our Hopper Speed", speed);
    }
}
