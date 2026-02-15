package frc.robot.subsystems.Intake;

import org.parts3492.partslib.PARTsUnit;
import org.parts3492.partslib.PARTsUnit.PARTsUnitType;

public class IntakeSim extends Intake {

    @Override
    public void setIntakeSpeed(double speed) {
    }

    @Override
    public void setPivotSpeed(double speed) {
    }

    @Override
    public void setPivotPosition(PARTsUnit position) {
    }

    @Override
    public double getIntakeSpeed() {
        return 0;
    }

    @Override
    public PARTsUnit getPivotPosition() {
        return new PARTsUnit(0, PARTsUnitType.Angle);
    }
    
}
