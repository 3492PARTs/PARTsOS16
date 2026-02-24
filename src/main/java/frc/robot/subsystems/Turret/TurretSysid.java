package frc.robot.subsystems.Turret;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class TurretSysid extends TurretPhys {

    private MutVoltage appliedVoltage;

    private MutAngle pivotAngle;

    private MutAngularVelocity pivotVelocity;

    private SysIdRoutine routine;
    
    public TurretSysid(Supplier<Pose2d> robotPoseSupplier) {
        super(robotPoseSupplier);

        appliedVoltage = Volts.mutable(0);

        pivotAngle = Units.Radian.mutable(0);

        pivotVelocity = Units.RadiansPerSecond.mutable(0);

        routine = new SysIdRoutine(
                new SysIdRoutine.Config(),
                new SysIdRoutine.Mechanism(
                        (voltage) -> this.setVoltage(voltage.in(Volts)),

                        log -> {
                            // Record a frame for the shooter motor.
                            log.motor("turret")
                                    .voltage(
                                            appliedVoltage.mut_replace(
                                                    super.turretMotor.getStatorCurrent().getValueAsDouble(), Volts))
                                    .angularPosition(pivotAngle.mut_replace(
                                            getAngle() * Math.PI / 180, Radians))
                                    .angularVelocity(
                                            pivotVelocity.mut_replace(super.turretMotor.getVelocity().getValueAsDouble(), RotationsPerSecond));
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
