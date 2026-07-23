package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.AutoLog;
import org.parts3492.partslib.PARTsUnit;

public interface IntakeIO {
  @AutoLog
  public static class IntakeInputs {
    public boolean connected = false;
    public double appliedVolts = 0.0;
    public double[] currentAmps = new double[0];
  }

  public default void updateInputs(IntakeInputs inputs) {}

  public default void setSpeed(double speed) {}

  public default double getSpeed() {
    return 0;
  }

  public default double getRPM() {
    return 0;
  }

  public default void setVoltage(double speed) {}

  public default double getVoltage() {
    return 0;
  }

  public default PARTsUnit getLinearPosition() {
    return null;
  }
}
