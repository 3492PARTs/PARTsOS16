package frc.robot.subsystems.Intake;

import org.parts3492.partslib.PARTsUnit;
import org.parts3492.partslib.PARTsUnit.PARTsUnitType;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.constants.IntakeConstants;

/**
 * The physical intake subsystem.<p>
 * WARNING: The pivot arm MUST be in home position when the robot starts.
 */
public class IntakePhys extends Intake {
    protected final TalonFX intakeMotor;
    protected final TalonFX pivotMotor;

    public IntakePhys() {
        super();
        TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
        intakeConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        TalonFXConfiguration pivotConfig = new TalonFXConfiguration();

        pivotConfig.CurrentLimits.SupplyCurrentLimit = 70;
        pivotConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        intakeMotor = new TalonFX(IntakeConstants.INTAKE_MOTOR_ID, IntakeConstants.CAN_BUS_NAME);
        intakeMotor.getConfigurator().apply(intakeConfig);
        intakeMotor.setNeutralMode(NeutralModeValue.Brake);

        pivotMotor = new TalonFX(IntakeConstants.PIVOT_MOTOR_ID, IntakeConstants.CAN_BUS_NAME);
        pivotMotor.getConfigurator().apply(pivotConfig);
        pivotMotor.setNeutralMode(NeutralModeValue.Brake);

        // Home the pivot position.
        pivotMotor.getConfigurator().setPosition(0);
    
    }

    @Override
    public void outputTelemetry() {
        super.outputTelemetry();
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

    @Override
    public void setPivotSpeed(double speed) {
        pivotMotor.set(speed);
    }

    @Override
    public void setPivotVoltage(double voltage) {
        pivotMotor.setVoltage(voltage);
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
    public double getPivotRotationSpeed() {
        return pivotMotor.getVelocity().getValueAsDouble();
    }

    @Override
    public PARTsUnit getPivotAngle() {
        return new PARTsUnit(pivotMotor.getPosition().getValueAsDouble() / IntakeConstants.PIVOT_GEAR_RATIO, PARTsUnitType.Rotations);
    }
}
