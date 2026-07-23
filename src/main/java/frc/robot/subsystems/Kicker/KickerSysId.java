package frc.robot.subsystems.Kicker;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.KickerConstants;

import org.parts3492.partslib.PARTsUnit.PARTsUnitType;

public class KickerSysId extends Kicker {
    
     private MutVoltage appliedVoltage;

    private MutDistance kickerPosition;

    private MutLinearVelocity kickerVelocity;

    private SysIdRoutine routine;


    public KickerSysId(KickerIO io) {
        super(io);

        appliedVoltage = Volts.mutable(0);

        kickerPosition = Inches.mutable(0);

        kickerVelocity = InchesPerSecond.mutable(0);

        routine = new SysIdRoutine(
                new SysIdRoutine.Config(), //ElevatorConstants.kSysIDConfig,
                new SysIdRoutine.Mechanism(
                        (Voltage v) -> super.setVoltage(v.in(Volts)),
                        (log) -> {
                            log.motor("shootermotor1")
                                    .voltage(appliedVoltage.mut_replace(
                                            io.getVoltage(), Volts))
                                    .linearPosition(kickerPosition.mut_replace(
                                            io.getLinearPosition().to(PARTsUnitType.Inch), Inches))
                                    .linearVelocity(kickerVelocity.mut_replace(
                                            (super.getRPM() * Math.PI * KickerConstants.KICKER_WHEEL_RADIUS.to(PARTsUnitType.Inch) * 2) / 60, InchesPerSecond));
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
