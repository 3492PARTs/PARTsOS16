package frc.robot.subsystems.Shooter;

public class ShooterSim extends Shooter {

    @Override
    protected void setSpeed(double speed) {
        partsNT.putDouble("Our Shooter Speed", speed);
    }
    
}
