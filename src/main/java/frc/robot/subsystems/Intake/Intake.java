package frc.robot.subsystems.Intake;

import org.parts3492.partslib.PARTsUnit;
import org.parts3492.partslib.PARTsUnit.PARTsUnitType;
import org.parts3492.partslib.command.PARTsSubsystem;

import frc.robot.constants.RobotConstants;
import frc.robot.states.IntakeState;

public abstract class Intake extends PARTsSubsystem {

    IntakeState state = IntakeState.IDLE;

    public Intake() {
        super("Intake");

        if (RobotConstants.DEBUGGING) {
            partsNT.putDouble("Intake Speed", 0);
            partsNT.putDouble("Pivot Speed", 0);
        }
    }

    @Override
    public void outputTelemetry() {
        partsNT.putDouble("Pivot Position", getPivotPosition().to(PARTsUnitType.Angle));
        partsNT.putDouble("Current Intake Speed", getIntakeSpeed());
        partsNT.putString("Intake State", state.toString());
    }

    @Override
    public void stop() {
        state = IntakeState.DISABLED;
    }

    @Override
    public void reset() {
        state = IntakeState.IDLE;
    }

    @Override
    public void periodic() {
        if (RobotConstants.DEBUGGING) {
            setIntakeSpeed(partsNT.getDouble("Intake Speed"));
            setPivotSpeed(partsNT.getDouble("Pivot Speed"));
        }
    }

    @Override
    public void log() {
        partsLogger.logDouble("Pivot Position", getPivotPosition().to(PARTsUnitType.Angle));
        partsLogger.logDouble("Intake Speed", getIntakeSpeed());
        partsLogger.logString("Intake State", state.toString());
    }

    public abstract void setIntakeSpeed(double speed);

    public abstract void setPivotSpeed(double speed);

    public abstract void setPivotPosition(PARTsUnit position);

    public abstract double getIntakeSpeed();

    public abstract PARTsUnit getPivotPosition();
}
