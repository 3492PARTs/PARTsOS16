package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.ShooterConstants;
import org.parts3492.partslib.PARTsUnit.PARTsUnitType;

public class ShooterSysid extends ShooterPhys {
    
     private MutVoltage appliedVoltage;

    private MutDistance shooterPosition;

    private MutLinearVelocity shooterVelocity;

    private SysIdRoutine routine;


    public ShooterSysid() {
        super();

        appliedVoltage = Volts.mutable(0);

        shooterPosition = Inches.mutable(0);

        shooterVelocity = InchesPerSecond.mutable(0);

        routine = new SysIdRoutine(
                new SysIdRoutine.Config(), //ElevatorConstants.kSysIDConfig,
                new SysIdRoutine.Mechanism(
                        (Voltage v) -> super.setVoltage(v.in(Volts)),
                        (log) -> {
                            log.motor("shootermotor1")
                                    .voltage(appliedVoltage.mut_replace(
                                            super.leftMotor.getStatorCurrent().getValueAsDouble(), Volts))
                                    .linearPosition(shooterPosition.mut_replace(
                                            super.leftMotor.getPosition().getValueAsDouble() * Math.PI * ShooterConstants.SHOOTER_WHEEL_RADIUS.to(PARTsUnitType.Inch) * 2, Inches))
                                    .linearVelocity(shooterVelocity.mut_replace(
                                            (super.getRPM() * Math.PI * ShooterConstants.SHOOTER_WHEEL_RADIUS.to(PARTsUnitType.Inch) * 2) / 60, InchesPerSecond));
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

