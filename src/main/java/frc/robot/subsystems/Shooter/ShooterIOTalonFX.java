package frc.robot.subsystems.Shooter;

import org.parts3492.partslib.PARTsUnit;
import org.parts3492.partslib.PARTsUnit.PARTsUnitType;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.constants.ShooterConstants;

public class ShooterIOTalonFX implements ShooterIO {

  protected final TalonFX leftMotor;
  protected final TalonFX rightMotor;

  public ShooterIOTalonFX(int leftId, int rightId) {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    config.CurrentLimits.SupplyCurrentLimit = 70;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLowerLimit = 40;
    config.CurrentLimits.SupplyCurrentLowerTime = 1.0;

    config.CurrentLimits.StatorCurrentLimit = 120;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    leftMotor = new TalonFX(ShooterConstants.LEFT_MOTOR_ID, ShooterConstants.CAN_BUS_NAME);
    leftMotor.getConfigurator().apply(config);

    rightMotor = new TalonFX(ShooterConstants.RIGHT_MOTOR_ID, ShooterConstants.CAN_BUS_NAME);
    rightMotor.setControl(
        new Follower(ShooterConstants.LEFT_MOTOR_ID, MotorAlignmentValue.Opposed));

    leftMotor.setNeutralMode(NeutralModeValue.Coast);
    rightMotor.setNeutralMode(NeutralModeValue.Coast);
  }

  @Override
  public void updateInputs(ShooterInputs inputs) {
    inputs.leftConnected = leftMotor.isConnected();
    inputs.rightConnected = rightMotor.isConnected();
    inputs.RPM = leftMotor.getVelocity().getValueAsDouble() * 60;
    inputs.velocityRadPerSec = leftMotor.getPosition().getValueAsDouble();
    inputs.appliedVolts = leftMotor.getMotorVoltage().getValueAsDouble();
    inputs.currentAmps = new double[] {leftMotor.getStatorCurrent().getValueAsDouble()};
  }

  @Override
  public void setRPM(double RPM) {}

  @Override
  public void setVoltage(double volts) {
    leftMotor.setVoltage(volts);
  }

  @Override
  public void stop() {
    leftMotor.setVoltage(0);
  }

  @Override
  public void setSpeed(double speed) {
    leftMotor.set(speed);
  }

  @Override
  public double getVoltage() {
    return leftMotor.getSupplyVoltage().getValueAsDouble();
  }

  @Override
  public PARTsUnit getLinearPosition() {
      return new PARTsUnit(leftMotor.getPosition().getValueAsDouble() * Math.PI * ShooterConstants.SHOOTER_WHEEL_RADIUS.to(PARTsUnitType.Inch) * 2, PARTsUnitType.Inch);
  }
}
