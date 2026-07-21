package frc.robot.subsystems.Hopper;

import org.littletonrobotics.junction.AutoLog;

public interface HopperIO {
  @AutoLog
  public static class HopperInputs {
    public boolean connected = false;
    public double appliedVolts = 0.0;
    public double[] currentAmps = new double[0];
  }

  public default void updateInputs(HopperInputs inputs) {}

  public default void setSpeed(double speed) {}

  public default double getRPM() {
    return 0;
  }

  public default void setVoltage(double voltage) {}

  public default double getVoltage() {
    return 0;
  }
}
