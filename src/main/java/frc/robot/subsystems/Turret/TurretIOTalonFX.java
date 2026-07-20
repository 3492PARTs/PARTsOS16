package frc.robot.subsystems.Turret;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.constants.TurretConstants;

public class TurretIOTalonFX implements TurretIO {
  protected final TalonFX turretMotor;

  public TurretIOTalonFX(int turretid) {
    turretMotor = new TalonFX(turretid, TurretConstants.CAN_BUS_NAME);
    TalonFXConfiguration config = new TalonFXConfiguration();

    /*absEncoder = new PARTsThroughBoreEncoder(TurretConstants.TURRET_ENCODER_PORT,
    TurretConstants.TURRET_OFFSET_ANGLE.to(PARTsUnitType.Angle));*/

    config.CurrentLimits.SupplyCurrentLimit = 30;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLowerTime = 0;

    config.CurrentLimits.StatorCurrentLimit = 80;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    turretMotor.getConfigurator().apply(config);
    turretMotor.getConfigurator().setPosition(0);
    turretMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  @Override
  public void updateInputs(Inputs inputs) {
    inputs.connected = turretMotor.isConnected();
    inputs.angle = getAngle();
    inputs.velocityRadPerSec = turretMotor.getVelocity().getValueAsDouble();
    inputs.appliedVolts = turretMotor.getMotorVoltage().getValueAsDouble();
    inputs.currentAmps = new double[] {turretMotor.getStatorCurrent().getValueAsDouble()};
  }

  @Override
  public double getAngle() {
    return turretMotor.getPosition().getValueAsDouble() * 360 / TurretConstants.TURRET_GEAR_RATIO;
    // return absEncoder.getAccumulatedAngle().getValue() / TurretConstants.TURRET_GEAR_RATIO;
  }
}
