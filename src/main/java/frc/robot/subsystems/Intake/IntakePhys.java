package frc.robot.subsystems.Intake;

import static edu.wpi.first.units.Units.Rotations;

import org.parts3492.partslib.PARTsUnit;
import org.parts3492.partslib.PARTsUnit.PARTsUnitType;
import org.parts3492.partslib.network.PARTsNT;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.motorcontrol.Talon;

import frc.robot.constants.IntakeConstants;

public class IntakePhys extends Intake {
    TalonFX intakeMotor;
    TalonFX pivotMotor;

    public IntakePhys() {
        intakeMotor = new TalonFX(IntakeConstants.INTAKE_MOTOR_ID);
        pivotMotor = new TalonFX(IntakeConstants.PIVOT_MOTOR_ID);
    }

    @Override
    public void setIntakeSpeed(double speed) {
        intakeMotor.set(speed);
    }

    @Override
    public void setPivotPosition(PARTsUnit position) {
        pivotMotor.setPosition(position.to(PARTsUnitType.Rotations));
    }

    @Override
    public double getIntakeSpeed() {
        return intakeMotor.get();
    }

    @Override
    public PARTsUnit getPivotPosition() {
        return new PARTsUnit(pivotMotor.getPosition().getValueAsDouble(), PARTsUnitType.Rotations);
    }

        @Override
    public void outputTelemetry() {
        partsNT.putDouble("Pivot/Output", pivotMotor.getMotorOutputStatus().getValueAsDouble());
        partsNT.putDouble("Pivot/Current", pivotMotor.getStatorCurrent().getValueAsDouble());

        partsNT.putDouble("Intake/Output", intakeMotor.getMotorOutputStatus().getValueAsDouble());
        partsNT.putDouble("Intake/Current", intakeMotor.getStatorCurrent().getValueAsDouble());
    }

    @Override
    public void stop() {
        super.stop();
        intakeMotor.set(0);
        pivotMotor.set(0);
    }

    @Override
    public void reset() {
        super.reset();
    }

    @Override
    public void log() {
        super.log();
        partsLogger.logDouble("Pivot/Output", pivotMotor.getMotorOutputStatus().getValueAsDouble());
        partsLogger.logDouble("Pivot/Current", pivotMotor.getStatorCurrent().getValueAsDouble());
        
        partsLogger.logDouble("Intake/Output", intakeMotor.getMotorOutputStatus().getValueAsDouble());
        partsLogger.logDouble("Intake/Current", intakeMotor.getStatorCurrent().getValueAsDouble());
    }
}
