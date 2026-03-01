package frc.robot.subsystems.Intake;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.parts3492.partslib.PARTsUnit.PARTsUnitType;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class IntakeSysid extends IntakePhys {

    private MutVoltage appliedVoltage;

    private MutAngle pivotAngle;

    private MutAngularVelocity pivotVelocity;

    private SysIdRoutine routine;

    public IntakeSysid() {
        super();

        appliedVoltage = Volts.mutable(0);

        pivotAngle = Units.Radian.mutable(0);

        pivotVelocity = Units.RadiansPerSecond.mutable(0);

        routine = new SysIdRoutine(
                new SysIdRoutine.Config(),
                new SysIdRoutine.Mechanism(
                        (voltage) -> this.setPivotVoltage(voltage.in(Volts)),

                        log -> {
                            // Record a frame for the shooter motor.
                            log.motor("pivotarm")
                                    .voltage(
                                            appliedVoltage.mut_replace(
                                                    super.pivotMotor.getStatorCurrent().getValueAsDouble(), Volts))
                                    .angularPosition(pivotAngle.mut_replace(
                                            getPivotAngle().toPARTsUnit(PARTsUnitType.Angle).to(PARTsUnitType.Radian), Radians))
                                    .angularVelocity(
                                            pivotVelocity.mut_replace(getPivotRotationSpeed(), RotationsPerSecond));
                        },
                        this));
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return routine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return routine.dynamic(direction);
    }
}
