package frc.robot.subsystems.Turret;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.subsystems.Drivetrain.PARTsDrivetrain;

public class TurretSim extends Turret {
    //protected final TalonFX turretMotor;

    double voltage = 0;
    double angle = 0;

    public TurretSim(Supplier<Pose2d> robotPoseSupplier, PARTsDrivetrain drivetrain) {
        super(robotPoseSupplier, drivetrain);
    }

    @Override
    protected void setSpeed(double speed) {
        voltage = speed * RobotController.getBatteryVoltage();
    }

    @Override
    protected void setVoltage(double voltage) {
        this.voltage = voltage;
    }

    @Override
    protected double getVoltage() {
        return voltage;
    }

    @Override
    protected double getAngle() {
        return angle;
    }

    @Override
    public void simulationPeriodic() {
        super.simulationPeriodic();
        angle = getAngleToTarget(getTargetPose());
    }
}
