package frc.robot.subsystems.Hopper;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.constants.HopperConstants;

public class HopperPhys extends Hopper {
    protected final TalonFX hopperMotor;

    public HopperPhys() {
        super();
        TalonFXConfiguration hopperConfig = new TalonFXConfiguration();
        hopperConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        hopperMotor = new TalonFX(HopperConstants.HOPPER_MOTOR_ID);
        hopperMotor.getConfigurator().apply(hopperConfig);
    }

    @Override
    public void outputTelemetry() {
        partsNT.putDouble("Output", hopperMotor.getStatorCurrent().getValueAsDouble());
        partsNT.putDouble("Current", hopperMotor.getSupplyCurrent().getValueAsDouble());
    }

    @Override
    protected void setSpeed(double speed) {
        hopperMotor.set(speed);
    }

    @Override
    public void periodic() {
        super.periodic();
    }

    @Override
    public void log() {
        partsLogger.logDouble("Output", hopperMotor.getStatorCurrent().getValueAsDouble());
        partsLogger.logDouble("Current", hopperMotor.getSupplyCurrent().getValueAsDouble());
    }
}
