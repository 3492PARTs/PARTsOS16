package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.littletonrobotics.junction.AutoLog;
import org.parts3492.partslib.PARTsUnit;

public interface PivotIO {
  @AutoLog
  public static class PivotInputs {
    public boolean connected = false;
    public double angle = 0;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double[] currentAmps = new double[0];
  }

  public default void updateInputs(PivotInputs inputs) {}

  public default void setSpeed(double speed) {}

  public default void setVoltage(double voltage) {}

  public default double getPivotRotationSpeed() {
    return 0;
  }

  public default PARTsUnit getPivotRotations() {
    return new PARTsUnit(0, null);
  }

  public default Command oneNinetyArm() {
    return new WaitCommand(0);
  }

  public default Command zeroArm() {
    return new WaitCommand(0);
  }

  public default double getVoltage() {
    return 0;
  }
}
