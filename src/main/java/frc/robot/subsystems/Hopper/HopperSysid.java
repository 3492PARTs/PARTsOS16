package frc.robot.subsystems.Hopper;

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
import frc.robot.constants.KickerConstants;

import org.parts3492.partslib.PARTsUnit.PARTsUnitType;

public class HopperSysid extends HopperPhys {
    
     private MutVoltage appliedVoltage;

    private MutDistance hopperPosition;

    private MutLinearVelocity hopperVelocity;

    private SysIdRoutine routine;


    public HopperSysid() {
        super();

        appliedVoltage = Volts.mutable(0);

        hopperPosition = Inches.mutable(0);

        hopperVelocity = InchesPerSecond.mutable(0);

        routine = new SysIdRoutine(
                new SysIdRoutine.Config(), //ElevatorConstants.kSysIDConfig,
                new SysIdRoutine.Mechanism(
                        (Voltage v) -> super.setVoltage(v.in(Volts)),
                        (log) -> {
                            log.motor("HopperMotor")
                                    .voltage(appliedVoltage.mut_replace(
                                            super.hopperMotor.getMotorVoltage().getValueAsDouble(), Volts))
                                    .linearPosition(hopperPosition.mut_replace(
                                            super.hopperMotor.getPosition().getValueAsDouble() * Math.PI * HopperConstants.HOPPER_ROLLER_RADIUS.to(PARTsUnitType.Inch) * 2, Inches))
                                    .linearVelocity(hopperVelocity.mut_replace(
                                            (super.getRPM() * Math.PI * HopperConstants.HOPPER_ROLLER_RADIUS.to(PARTsUnitType.Inch) * 2) / 60, InchesPerSecond));
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
