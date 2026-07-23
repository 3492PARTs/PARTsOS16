package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.constants.HopperConstants;
import frc.robot.constants.IntakeConstants;
import org.parts3492.partslib.PARTsUnit;
import org.parts3492.partslib.PARTsUnit.PARTsUnitType;

public class IntakeIOTalonFX implements IntakeIO {
  protected final TalonFX intakeMotor;

  public IntakeIOTalonFX() {
    TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
    intakeConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    intakeConfig.CurrentLimits.SupplyCurrentLimit = 30;
    intakeConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    intakeConfig.CurrentLimits.SupplyCurrentLowerTime = 0;

    intakeConfig.CurrentLimits.StatorCurrentLimit = 60;
    intakeConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    intakeMotor = new TalonFX(IntakeConstants.INTAKE_MOTOR_ID, IntakeConstants.CAN_BUS_NAME);
    intakeMotor.getConfigurator().apply(intakeConfig);
    intakeMotor.setNeutralMode(NeutralModeValue.Coast);
  }

  @Override
  public void updateInputs(IntakeInputs inputs) {
    // TODO Auto-generated method stub
    IntakeIO.super.updateInputs(inputs);
  }

  @Override
  public void setSpeed(double speed) {
    intakeMotor.set(speed);
  }

  @Override
  public double getSpeed() {
    return intakeMotor.get();
  }

  @Override
  public double getRPM() {
    return intakeMotor.getVelocity().getValueAsDouble() * 60;
  }

  @Override
  public void setVoltage(double speed) {
    intakeMotor.setVoltage(speed);
  }

  @Override
  public double getVoltage() {
    return intakeMotor.getMotorVoltage().getValueAsDouble();
  }

  @Override
  public PARTsUnit getLinearPosition() {
    return new PARTsUnit(
        intakeMotor.getPosition().getValueAsDouble()
            * Math.PI
            * HopperConstants.HOPPER_ROLLER_RADIUS.to(PARTsUnitType.Inch)
            * 2,
        PARTsUnitType.Inch);
  }
}
