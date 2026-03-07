package frc.robot.subsystems.Intake;

import org.parts3492.partslib.PARTsUnit;
import org.parts3492.partslib.PARTsUnit.PARTsUnitType;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.RobotContainer;
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

        intakeConfig.CurrentLimits.SupplyCurrentLimit = 70;
        intakeConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

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
        partsNT.putDouble("Pivot/Output", pivotMotor.getMotorOutputStatus().getValueAsDouble(), RobotContainer.debug || super.debug);
        partsNT.putDouble("Pivot/Current", pivotMotor.getStatorCurrent().getValueAsDouble(), RobotContainer.debug || super.debug);

        partsNT.putDouble("Intake/Output", intakeMotor.getMotorOutputStatus().getValueAsDouble(), RobotContainer.debug || super.debug);
        partsNT.putDouble("Intake/Current", intakeMotor.getStatorCurrent().getValueAsDouble(), RobotContainer.debug || super.debug);
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
        partsLogger.logDouble("Pivot/Output", pivotMotor.getMotorOutputStatus().getValueAsDouble(), RobotContainer.debug || super.debug);
        partsLogger.logDouble("Pivot/Current", pivotMotor.getStatorCurrent().getValueAsDouble(), RobotContainer.debug || super.debug);
        
        partsLogger.logDouble("Intake/Output", intakeMotor.getMotorOutputStatus().getValueAsDouble(), RobotContainer.debug || super.debug);
        partsLogger.logDouble("Intake/Current", intakeMotor.getStatorCurrent().getValueAsDouble(), RobotContainer.debug || super.debug);
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
        return pivotMotor.getVelocity().getValueAsDouble() / IntakeConstants.PIVOT_GEAR_RATIO;
    }

    @Override
    public PARTsUnit getPivotRotations() {
        return new PARTsUnit(pivotMotor.getPosition().getValueAsDouble() / IntakeConstants.PIVOT_GEAR_RATIO, PARTsUnitType.Rotations);
    }
}
