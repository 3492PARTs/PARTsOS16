package frc.robot.subsystems.Hopper;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.constants.HopperConstants;

public class HopperPhys extends Hopper {
    protected final TalonFX hopperMotor;

    public HopperPhys() {
        super();
        TalonFXConfiguration hopperConfig = new TalonFXConfiguration();
        hopperConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        hopperMotor = new TalonFX(HopperConstants.HOPPER_MOTOR_ID, HopperConstants.CAN_BUS_NAME);
        hopperMotor.getConfigurator().apply(hopperConfig);
        hopperMotor.setNeutralMode(NeutralModeValue.Brake);
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
