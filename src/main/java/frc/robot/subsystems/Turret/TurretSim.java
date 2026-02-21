package frc.robot.subsystems.Turret;

import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Pose2d;

public class TurretSim extends Turret {
    //protected final TalonFX turretMotor;

    public TurretSim(Supplier<Pose2d> robotPoseSupplier) {
        super(robotPoseSupplier);
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
