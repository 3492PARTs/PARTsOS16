package frc.robot.subsystems.Turret;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.constants.TurretConstants;

public class TurretPhys extends Turret {
    protected final TalonFX turretMotor;

    public TurretPhys(Supplier<Pose2d> robotPoseSupplier) {
        super(robotPoseSupplier);

        turretMotor = new TalonFX(TurretConstants.TURRET_MOTOR_ID, TurretConstants.CAN_BUS_NAME);
        TalonFXConfiguration turretConfig = new TalonFXConfiguration();
        turretMotor.getConfigurator().apply(turretConfig);
        turretMotor.getConfigurator().setPosition(0);
        turretMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    @Override
    public void outputTelemetry() {
        super.outputTelemetry();
        partsNT.putDouble("Current/Turret", turretMotor.getSupplyCurrent().getValueAsDouble());

        partsNT.putDouble("Output/Turret", turretMotor.getStatorCurrent().getValueAsDouble());
    }

    @Override
    public void periodic() {
        super.periodic();
    }

    @Override
    public void log() {
        super.log();
        partsLogger.logDouble("Current/Turret", turretMotor.getSupplyCurrent().getValueAsDouble());

        partsLogger.logDouble("Output/Turret", turretMotor.getStatorCurrent().getValueAsDouble());
    }

    @Override
    protected double getAngle() {
        return turretMotor.getPosition().getValueAsDouble() * 360 / TurretConstants.TURRET_GEAR_RATIO;
    }

    @Override
    protected void setSpeed(double speed) {
        turretMotor.set(speed);
    }

    @Override
    protected void setVoltage(double voltage) {
        turretMotor.setVoltage(voltage);
    }

    @Override
    protected double getVoltage() {
        return turretMotor.getSupplyCurrent().getValueAsDouble();
    }
}