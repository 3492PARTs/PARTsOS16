package frc.robot.subsystems.Turret;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.PowerDistribution;
import frc.robot.constants.TurretConstants;

public class TurretPhys extends Turret {
    protected final SparkMax turretMotor;

    protected final RelativeEncoder turretEncoder;

    public TurretPhys() {
        super();

        SparkMaxConfig turretConfig = new SparkMaxConfig();
        turretConfig.idleMode(IdleMode.kCoast);
        turretConfig.inverted(true);

        turretMotor = new SparkMax(TurretConstants.TURRET_MOTOR_ID,
                com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
        turretEncoder = turretMotor.getEncoder();
        turretMotor.configure(turretConfig, com.revrobotics.ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
    }

    @Override
    public void outputTelemetry() {
        super.outputTelemetry();
        partsNT.putDouble("Current/Turret", turretMotor.getOutputCurrent());

        partsNT.putDouble("Output/Turret", turretMotor.getAppliedOutput());
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
        return turretMotor.getBusVoltage();
    }

    @Override
    public void periodic() {
        super.periodic();
    }

    @Override
    public void log() {
        super.log();
        partsLogger.logDouble("Current/Turret", turretMotor.getOutputCurrent());

        partsLogger.logDouble("Output/Turret", turretMotor.getAppliedOutput());
    }

    @Override
    protected double getAngle() {
        return turretEncoder.getPosition() * 360 % 360;
    }
}