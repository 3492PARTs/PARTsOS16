package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.IntakeConstants;
import org.parts3492.partslib.PARTsUnit;
import org.parts3492.partslib.PARTsUnit.PARTsUnitType;
import org.parts3492.partslib.command.PARTsCommandUtils;

public class PivotIOTalonFX implements PivotIO {

  protected final TalonFX pivotMotor;
  ProfiledPIDController pivotPIDController;

  public PivotIOTalonFX() {
    TalonFXConfiguration pivotConfig = new TalonFXConfiguration();

    pivotConfig.CurrentLimits.SupplyCurrentLimit = 30;
    pivotConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    pivotConfig.CurrentLimits.SupplyCurrentLowerTime = 0;

    pivotConfig.CurrentLimits.StatorCurrentLimit = 100;
    pivotConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    pivotMotor = new TalonFX(IntakeConstants.PIVOT_MOTOR_ID, IntakeConstants.CAN_BUS_NAME);
    pivotMotor.getConfigurator().apply(pivotConfig);
    pivotMotor.setNeutralMode(NeutralModeValue.Coast);
  }

  @Override
  public void updateInputs(PivotInputs inputs) {
    // TODO Auto-generated method stub
    PivotIO.super.updateInputs(inputs);
  }

  @Override
  public void setPivotSpeed(double speed) {
    pivotMotor.set(speed);
  }

  @Override
  public void setPivotVoltage(double voltage) {
    pivotMotor.setVoltage(voltage);
  }

  @Override
  public double getPivotRotationSpeed() {
    return pivotMotor.getVelocity().getValueAsDouble() / IntakeConstants.PIVOT_GEAR_RATIO;
  }

  @Override
  public PARTsUnit getPivotRotations() {
    return new PARTsUnit(
        pivotMotor.getPosition().getValueAsDouble() / IntakeConstants.PIVOT_GEAR_RATIO,
        PARTsUnitType.Rotations);
  }

  @Override
  public Command oneNinetyArm() {
    return PARTsCommandUtils.setCommandName(
        "Intake.zeroArm",
        Commands.runOnce(
            () -> {
              pivotMotor
                  .getConfigurator()
                  .setPosition(
                      new PARTsUnit(198, PARTsUnitType.Angle).to(PARTsUnitType.Rotations)
                          * IntakeConstants.PIVOT_GEAR_RATIO);
              pivotPIDController.reset(getPivotRotations().to(PARTsUnitType.Angle));
            }));
  }

  public Command zeroArm() {
    return PARTsCommandUtils.setCommandName(
        "Intake.zeroArm",
        Commands.runOnce(
            () -> {
              pivotMotor
                  .getConfigurator()
                  .setPosition(
                      new PARTsUnit(0, PARTsUnitType.Angle).to(PARTsUnitType.Rotations)
                          * IntakeConstants.PIVOT_GEAR_RATIO);
              pivotPIDController.reset(getPivotRotations().to(PARTsUnitType.Angle));
            }));
  }
}
