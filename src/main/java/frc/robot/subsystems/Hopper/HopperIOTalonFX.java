package frc.robot.subsystems.Hopper;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.constants.HopperConstants;

public class HopperIOTalonFX implements HopperIO {
  protected final TalonFX hopperMotor;

  public HopperIOTalonFX() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    config.CurrentLimits.SupplyCurrentLimit = 30;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLowerTime = 0;

    config.CurrentLimits.StatorCurrentLimit = 40;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    hopperMotor = new TalonFX(HopperConstants.HOPPER_MOTOR_ID, HopperConstants.CAN_BUS_NAME);
    hopperMotor.getConfigurator().apply(config);
    hopperMotor.setNeutralMode(NeutralModeValue.Coast);
  }

  @Override
  public void setSpeed(double speed) {
    hopperMotor.set(speed);
  }

  @Override
  public double getRPM() {
    return hopperMotor.getVelocity().getValueAsDouble() * 60 / HopperConstants.HOPPER_GEAR_RATIO;
  }

  @Override
  public void setVoltage(double voltage) {
    hopperMotor.setVoltage(voltage);
  }

  @Override
  public double getVoltage() {
    return hopperMotor.getSupplyVoltage().getValueAsDouble();
  }

  @Override
  public void updateInputs(HopperInputs inputs) {
    inputs.connected = hopperMotor.isConnected();
    inputs.appliedVolts = hopperMotor.getMotorVoltage().getValueAsDouble();
    inputs.currentAmps = new double[] {hopperMotor.getStatorCurrent().getValueAsDouble()};
  }
}
