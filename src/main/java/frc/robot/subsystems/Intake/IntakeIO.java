package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeInputs {
    public boolean connected = false;
    public double appliedVolts = 0.0;
    public double[] currentAmps = new double[0];
  }

  public default void updateInputs(IntakeInputs inputs) {}

  public default void setIntakeSpeed(double speed) {}

  public default double getIntakeSpeed() {
    return 0;
  }

  public default double getIntakeRPM() {
    return 0;
  }

  public default void setIntakeVoltage(double speed) {}
}
