package frc.robot.subsystems.Intake;

import org.parts3492.partslib.PARTsUnit;
import org.parts3492.partslib.PARTsUnit.PARTsUnitType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class IntakeSim extends Intake {

    @Override
    public void setIntakeSpeed(double speed) {
    }

    @Override
    public void setPivotSpeed(double speed) {
    }

    @Override
    public double getIntakeSpeed() {
        return 0;
    }

    @Override
    public PARTsUnit getPivotRotations() {
        return new PARTsUnit(0, PARTsUnitType.Angle);
    }

    @Override
    public void setPivotVoltage(double voltage) {
    }

    @Override
    public double getPivotRotationSpeed() {
        return 0;
    }

    @Override
    public Command zeroArm() {
        return new WaitCommand(0);
    }
}
