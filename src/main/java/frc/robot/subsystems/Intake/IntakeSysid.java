package frc.robot.subsystems.Intake;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.HopperConstants;

import org.parts3492.partslib.PARTsUnit.PARTsUnitType;

public class IntakeSysid extends IntakePhys {
    
     private MutVoltage appliedVoltage;

    private MutDistance intakePosition;

    private MutLinearVelocity intakeVelocity;

    private SysIdRoutine routine;


    public IntakeSysid() {
        super();

        appliedVoltage = Volts.mutable(0);

        intakePosition = Inches.mutable(0);

        intakeVelocity = InchesPerSecond.mutable(0);

        //? The hopper roller and the intake roller are both the same, so the hopper roller is used here.
        routine = new SysIdRoutine(
                new SysIdRoutine.Config(), //ElevatorConstants.kSysIDConfig,
                new SysIdRoutine.Mechanism(
                        (Voltage v) -> super.setIntakeVoltage(v.in(Volts)),
                        (log) -> {
                            log.motor("IntakeMotor")
                                    .voltage(appliedVoltage.mut_replace(
                                            super.intakeMotor.getMotorVoltage().getValueAsDouble(), Volts))
                                    .linearPosition(intakePosition.mut_replace(
                                            super.intakeMotor.getPosition().getValueAsDouble() * Math.PI * HopperConstants.HOPPER_ROLLER_RADIUS.to(PARTsUnitType.Inch) * 2, Inches))
                                    .linearVelocity(intakeVelocity.mut_replace(
                                            (super.getIntakeRPM() * Math.PI * HopperConstants.HOPPER_ROLLER_RADIUS.to(PARTsUnitType.Inch) * 2) / 60, InchesPerSecond));
                        },
                        this));
    }

    @Override
    public void periodic() {
        //dummy to stop super periodic from running
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return routine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return routine.dynamic(direction);
    }
}
