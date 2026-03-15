package frc.robot.subsystems.Intake;

import org.parts3492.partslib.PARTsUnit;
import org.parts3492.partslib.PARTsUnit.PARTsUnitType;
import org.parts3492.partslib.command.PARTsCommandUtils;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.constants.IntakeConstants;

/**
 * The physical intake subsystem.
 * <p>
 * WARNING: The pivot arm MUST be in home position when the robot starts.
 */
public class IntakePhys extends Intake {
    protected final TalonFX intakeMotor;
    protected final TalonFX pivotMotor;

    public IntakePhys() {
        super();
        TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
        intakeConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        intakeConfig.CurrentLimits.SupplyCurrentLimit = 30;
        intakeConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        intakeConfig.CurrentLimits.SupplyCurrentLowerTime = 0;

        intakeConfig.CurrentLimits.StatorCurrentLimit = 100;
        intakeConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        TalonFXConfiguration pivotConfig = new TalonFXConfiguration();

        pivotConfig.CurrentLimits.SupplyCurrentLimit = 30;
        pivotConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        pivotConfig.CurrentLimits.SupplyCurrentLowerTime = 0;

        pivotConfig.CurrentLimits.StatorCurrentLimit = 100;
        pivotConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        intakeMotor = new TalonFX(IntakeConstants.INTAKE_MOTOR_ID, IntakeConstants.CAN_BUS_NAME);
        intakeMotor.getConfigurator().apply(intakeConfig);
        intakeMotor.setNeutralMode(NeutralModeValue.Coast);

        pivotMotor = new TalonFX(IntakeConstants.PIVOT_MOTOR_ID, IntakeConstants.CAN_BUS_NAME);
        pivotMotor.getConfigurator().apply(pivotConfig);
        pivotMotor.setNeutralMode(NeutralModeValue.Brake);

        // Home the pivot position.
        pivotMotor.getConfigurator().setPosition(0);

    }

    @Override
    public void outputTelemetry() {
        super.outputTelemetry();
        partsNT.putDouble("Pivot/Output", pivotMotor.getMotorOutputStatus().getValueAsDouble(),
                RobotContainer.debug || super.debug);
        partsNT.putDouble("Pivot/Current", pivotMotor.getStatorCurrent().getValueAsDouble(),
                RobotContainer.debug || super.debug);

        partsNT.putDouble("Intake/Output", intakeMotor.getMotorOutputStatus().getValueAsDouble(),
                RobotContainer.debug || super.debug);
        partsNT.putDouble("Intake/Current", intakeMotor.getStatorCurrent().getValueAsDouble(),
                RobotContainer.debug || super.debug);
    }

    @Override
    public void reset() {
        super.reset();
    }

    @Override
    public void log() {
        super.log();
        partsLogger.logDouble("Pivot/Output", pivotMotor.getMotorOutputStatus().getValueAsDouble(),
                RobotContainer.debug || super.debug);
        partsLogger.logDouble("Pivot/Current", pivotMotor.getStatorCurrent().getValueAsDouble(),
                RobotContainer.debug || super.debug);

        partsLogger.logDouble("Intake/Output", intakeMotor.getMotorOutputStatus().getValueAsDouble(),
                RobotContainer.debug || super.debug);
        partsLogger.logDouble("Intake/Current", intakeMotor.getStatorCurrent().getValueAsDouble(),
                RobotContainer.debug || super.debug);
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
        return new PARTsUnit(pivotMotor.getPosition().getValueAsDouble() / IntakeConstants.PIVOT_GEAR_RATIO,
                PARTsUnitType.Rotations);
    }

    @Override 
    public Command zeroArm() {
        return PARTsCommandUtils.setCommandName("Intake.zeroArm", Commands.runOnce(() -> {
            pivotMotor.getConfigurator().setPosition(190);
        }));
    }
}
