package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.PowerDistribution;
import frc.robot.constants.ShooterConstants;

public class ShooterPhys extends Shooter {
    protected final TalonFX leftMotor;
    protected final TalonFX rightMotor;

    public ShooterPhys() {
        super();

        leftMotor = new TalonFX(ShooterConstants.LEFT_MOTOR_ID);
        rightMotor = new TalonFX(ShooterConstants.RIGHT_MOTOR_ID);
    }

    @Override
    public void outputTelemetry() {
        super.outputTelemetry();
        partsNT.putDouble("Current/Left", leftMotor.getSupplyCurrent().getValueAsDouble());
        partsNT.putDouble("Current/Right", rightMotor.getSupplyCurrent().getValueAsDouble());

        partsNT.putDouble("Output/Left", leftMotor.getStatorCurrent().getValueAsDouble());
        partsNT.putDouble("Output/Right", rightMotor.getStatorCurrent().getValueAsDouble());
    }

    @Override
    protected void setSpeed(double speed) {
        leftMotor.set(speed);
    }

    @Override
    protected double getRPM() {
        return leftMotor.getVelocity().getValueAsDouble() / 60;
    }

    @Override
    protected void setVoltage(double voltage) {
        leftMotor.setVoltage(voltage);
    }

    @Override
    protected double getVoltage() {
        return leftMotor.getSupplyVoltage().getValueAsDouble();
    }

    @Override
    public void periodic() {
        super.periodic();
    }

    @Override
    public void log() {
        super.log();
        partsLogger.logDouble("Current/Left", leftMotor.getSupplyCurrent().getValueAsDouble());
        partsLogger.logDouble("Current/Right", rightMotor.getSupplyCurrent().getValueAsDouble());

        partsLogger.logDouble("Output/Left", leftMotor.getStatorCurrent().getValueAsDouble());
        partsLogger.logDouble("Output/Right", rightMotor.getStatorCurrent().getValueAsDouble());
    }
}
