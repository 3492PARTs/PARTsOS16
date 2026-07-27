package frc.robot.subsystems.Kicker;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.constants.KickerConstants;
import org.parts3492.partslib.PARTsUnit;
import org.parts3492.partslib.PARTsUnit.PARTsUnitType;

public class KickerIOTalonFX implements KickerIO {
  protected final TalonFX kickerMotor;

  public KickerIOTalonFX(int kickerID) {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    config.CurrentLimits.SupplyCurrentLimit = 30;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLowerTime = 0;

    config.CurrentLimits.StatorCurrentLimit = 45;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    kickerMotor = new TalonFX(kickerID, KickerConstants.CAN_BUS_NAME);
    kickerMotor.getConfigurator().apply(config);
    kickerMotor.setNeutralMode(NeutralModeValue.Coast);
  }

  @Override
  public void updateInputs(KickerInputs inputs) {
    inputs.connected = kickerMotor.isConnected();
    inputs.RPM = kickerMotor.getVelocity().getValueAsDouble() * 60;
    inputs.positionRadPerSec = kickerMotor.getPosition().getValueAsDouble();
    inputs.appliedVolts = kickerMotor.getMotorVoltage().getValueAsDouble();
    inputs.currentAmps = new double[] {kickerMotor.getStatorCurrent().getValueAsDouble()};
  }

  @Override
  public void setSpeed(double speed) {
    kickerMotor.set(speed);
  }

  @Override
  public double getRPM() {
    return kickerMotor.getVelocity().getValueAsDouble() * 60 / KickerConstants.KICKER_GEAR_RATIO;
  }

  @Override
  public void setVoltage(double voltage) {
    kickerMotor.setVoltage(voltage);
  }

  @Override
  public double getVoltage() {
    return kickerMotor.getSupplyVoltage().getValueAsDouble();
  }

  @Override
  public PARTsUnit getLinearPosition() {
    return new PARTsUnit(
        kickerMotor.getPosition().getValueAsDouble()
            * Math.PI
            * KickerConstants.KICKER_WHEEL_RADIUS.to(PARTsUnitType.Inch)
            * 2,
        PARTsUnitType.Inch);
  }
}
