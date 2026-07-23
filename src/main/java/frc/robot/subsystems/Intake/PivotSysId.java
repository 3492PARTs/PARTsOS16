package frc.robot.subsystems.Intake;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.Logger;

public class PivotSysId extends Intake {

  private MutVoltage appliedVoltage;

  private MutAngle pivotAngle;

  private MutAngularVelocity pivotVelocity;

  private SysIdRoutine routine;

  public PivotSysId(IntakeIO io, PivotIO pivotIO) {
    super(io, pivotIO);

    appliedVoltage = Volts.mutable(0);

    pivotAngle = Units.Radian.mutable(0);

    pivotVelocity = Units.RadiansPerSecond.mutable(0);

    routine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Pivot/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> this.setPivotVoltage(voltage.in(Volts)),
                log -> {
                  // Record a frame for the shooter motor.
                  log.motor("pivotarm")
                      .voltage(appliedVoltage.mut_replace(pivotIO.getVoltage(), Volts))
                      .angularPosition(
                          pivotAngle.mut_replace(-getPivotRotations().getValue(), Rotations))
                      .angularVelocity(
                          pivotVelocity.mut_replace(getPivotRotationSpeed(), RotationsPerSecond));
                },
                this));
  }

  @Override
  public void periodic() {
    // dummy to stop super periodic from running
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return routine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return routine.dynamic(direction);
  }
}
