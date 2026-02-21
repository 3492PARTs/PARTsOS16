package frc.robot.subsystems.Intake;

import org.parts3492.partslib.PARTsUnit;
import org.parts3492.partslib.PARTsUnit.PARTsUnitType;
import org.parts3492.partslib.command.PARTsCommandUtils;
import org.parts3492.partslib.command.PARTsSubsystem;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.states.IntakeState;
import frc.robot.states.ShooterState;

public abstract class Intake extends PARTsSubsystem {

    IntakeState intakeState = IntakeState.IDLE;

    PIDController intakePIDController;
    SimpleMotorFeedforward intakeFeedForward;

    public Intake() {
        super("Intake");

        if (RobotConstants.DEBUGGING) {
            partsNT.putDouble("Intake Speed", 0);
            partsNT.putDouble("Pivot Speed", 0);
        }

        intakePIDController = new PIDController(IntakeConstants.P, IntakeConstants.I, IntakeConstants.D);
        intakeFeedForward = new SimpleMotorFeedforward(IntakeConstants.S, IntakeConstants.V, IntakeConstants.A);
        intakePIDController.setTolerance(IntakeConstants.PID_THRESHOLD);

    }

    @Override
    public void outputTelemetry() {
        partsNT.putDouble("Pivot Position", getPivotAngle().to(PARTsUnitType.Angle));
        partsNT.putDouble("Current Intake Speed", getIntakeSpeed());
        partsNT.putString("Intake State", intakeState.toString());
    }

    @Override
    public void stop() {
        intakeState = IntakeState.DISABLED;
    }

    @Override
    public void reset() {
        intakeState = IntakeState.IDLE;
    }

    @Override
    public void periodic() {
        if (RobotConstants.DEBUGGING) {
            setIntakeSpeed(partsNT.getDouble("Intake Speed"));
            setPivotSpeed(partsNT.getDouble("Pivot Speed"));
        } else {
            switch (intakeState) {
                case DISABLED:
                case IDLE:
                    setIntakeSpeed(intakeState.getSpeed());
                    setPivotSpeed(0);
                    break;
                case INTAKING:
                case OUTTAKING:
                case SHOOTING:
                    setIntakeSpeed(intakeState.getSpeed());
                    intakePIDController.setSetpoint(intakeState.getAngle());
                    double pidCalc = intakePIDController.calculate(getPivotAngle().to(PARTsUnitType.Angle), intakeState.getAngle());
                    double ffCalc = intakeFeedForward.calculate(intakePIDController.getSetpoint());

                    //setPivotVoltage(pidCalc + ffCalc);
                    break;
                default:
                    setIntakeSpeed(0);
                    setPivotSpeed(0);
                    break;
            }
        }
    }

    @Override
    public void log() {
        partsLogger.logDouble("Pivot Position", getPivotAngle().to(PARTsUnitType.Angle));
        partsLogger.logDouble("Intake Speed", getIntakeSpeed());
        partsLogger.logString("Intake State", intakeState.toString());
    }

    public abstract void setIntakeSpeed(double speed);

    public abstract void setPivotSpeed(double speed);

    public abstract double getIntakeSpeed();

    public abstract PARTsUnit getPivotAngle();

    public abstract void setPivotVoltage(double voltage);

    public abstract double getPivotRotationSpeed();

    public Command intake() {
        return PARTsCommandUtils.setCommandName("Command Intake", this.runOnce(() -> {
            intakeState = IntakeState.INTAKING;
        }));
    }
}
