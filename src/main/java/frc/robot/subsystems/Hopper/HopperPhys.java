package frc.robot.subsystems.Hopper;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.RobotContainer;
import frc.robot.constants.HopperConstants;

public class HopperPhys extends Hopper {
    protected final TalonFX hopperMotor;

    public HopperPhys() {
        super();
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        config.CurrentLimits.SupplyCurrentLimit = 30;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLowerTime = 0;

        config.CurrentLimits.StatorCurrentLimit = 100;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        hopperMotor = new TalonFX(HopperConstants.HOPPER_MOTOR_ID, HopperConstants.CAN_BUS_NAME);
        hopperMotor.getConfigurator().apply(config);
        hopperMotor.setNeutralMode(NeutralModeValue.Coast);
    }

    @Override
    public void outputTelemetry() {
        super.outputTelemetry();
        partsNT.putDouble("Output", hopperMotor.getStatorCurrent().getValueAsDouble(),
                RobotContainer.debug || super.debug);
        partsNT.putDouble("Current", hopperMotor.getSupplyCurrent().getValueAsDouble(),
                RobotContainer.debug || super.debug);
    }

    @Override
    public void periodic() {
        super.periodic();
    }

    @Override
    public void log() {
        partsLogger.logDouble("Output", hopperMotor.getStatorCurrent().getValueAsDouble(),
                RobotContainer.debug || super.debug);
        partsLogger.logDouble("Current", hopperMotor.getSupplyCurrent().getValueAsDouble(),
                RobotContainer.debug || super.debug);
    }

    @Override
    protected void setSpeed(double speed) {
        hopperMotor.set(speed);
    }
}
