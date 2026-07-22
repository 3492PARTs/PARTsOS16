package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.constants.IntakeConstants;

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
  public void setIntakeSpeed(double speed) {
    intakeMotor.set(speed);
  }

  @Override
  public double getIntakeSpeed() {
    return intakeMotor.get();
  }

  @Override
  public double getIntakeRPM() {
    return intakeMotor.getVelocity().getValueAsDouble() * 60;
  }

  @Override
  public void setIntakeVoltage(double speed) {
    intakeMotor.setVoltage(speed);
  }
}
