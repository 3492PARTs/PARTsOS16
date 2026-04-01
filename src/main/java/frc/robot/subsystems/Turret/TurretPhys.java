package frc.robot.subsystems.Turret;

import java.util.function.Supplier;

import org.parts3492.partslib.PARTsUnit.PARTsUnitType;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.RobotContainer;
import frc.robot.constants.TurretConstants;
import frc.robot.subsystems.Drivetrain.PARTsDrivetrain;
import frc.robot.util.PARTsThroughBoreEncoder;

public class TurretPhys extends Turret {

    protected final TalonFX turretMotor;
    protected final PARTsThroughBoreEncoder absEncoder;

    public TurretPhys(Supplier<Pose2d> robotPoseSupplier, PARTsDrivetrain drivetrain) {
        super(robotPoseSupplier, drivetrain);

        turretMotor = new TalonFX(TurretConstants.TURRET_MOTOR_ID, TurretConstants.CAN_BUS_NAME);
        TalonFXConfiguration config = new TalonFXConfiguration();

        absEncoder = new PARTsThroughBoreEncoder(TurretConstants.TURRET_ENCODER_PORT,
                TurretConstants.TURRET_OFFSET_ANGLE.to(PARTsUnitType.Angle));

        config.CurrentLimits.SupplyCurrentLimit = 30;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLowerTime = 0;

        config.CurrentLimits.StatorCurrentLimit = 80;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        turretMotor.getConfigurator().apply(config);
        turretMotor.getConfigurator().setPosition(0);
        turretMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    @Override
    public void outputTelemetry() {
        super.outputTelemetry();
        partsNT.putDouble("Current/Turret", turretMotor.getSupplyCurrent().getValueAsDouble(),
                RobotContainer.debug || super.debug);

        partsNT.putDouble("Output/Turret", turretMotor.getStatorCurrent().getValueAsDouble(),
                RobotContainer.debug || super.debug);
    }

    @Override
    public void periodic() {
        super.periodic();
    }

    @Override
    public void log() {
        super.log();
        partsLogger.logDouble("Current/Turret", turretMotor.getSupplyCurrent().getValueAsDouble(),
                RobotContainer.debug || super.debug);

        partsLogger.logDouble("Output/Turret", turretMotor.getStatorCurrent().getValueAsDouble(),
                RobotContainer.debug || super.debug);
    }

    @Override
    protected double getAngle() {
        //return turretMotor.getPosition().getValueAsDouble() * 360 / TurretConstants.TURRET_GEAR_RATIO;
        return absEncoder.getAngleDeg() / TurretConstants.TURRET_GEAR_RATIO;
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