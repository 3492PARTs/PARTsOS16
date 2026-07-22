package frc.robot.subsystems.Kicker;

import org.littletonrobotics.junction.AutoLog;

public interface KickerIO {
  @AutoLog
  public static class KickerInputs {
    public boolean connected = false;
    public double RPM = 0.0;
    public double positionRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double[] currentAmps = new double[0];
  }

  public default void updateInputs(KickerInputs inputs) {}

  public default void setSpeed(double speed) {}

  public default double getRPM() {
    return 0;
  }

  public default void setVoltage(double voltage) {}

  public default double getVoltage() {
    return 0;
  }
}
