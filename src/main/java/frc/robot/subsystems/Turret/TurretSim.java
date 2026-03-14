package frc.robot.subsystems.Turret;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.Drivetrain.PARTsDrivetrain;

public class TurretSim extends Turret {
    //protected final TalonFX turretMotor;

    public TurretSim(Supplier<Pose2d> robotPoseSupplier, PARTsDrivetrain drivetrain) {
        super(robotPoseSupplier, drivetrain);
    }

    @Override
    protected void setSpeed(double speed) {
    }

    @Override
    protected void setVoltage(double voltage) {
    }

    @Override
    protected double getVoltage() {
        return 0;
    }

    @Override
    protected double getAngle() {
        return 0;
    }
}
