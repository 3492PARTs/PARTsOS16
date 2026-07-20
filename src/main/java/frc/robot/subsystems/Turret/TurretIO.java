package frc.robot.subsystems.Turret;

import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {
  @AutoLog
  public static class Inputs {
    public boolean connected = false;
    public double angle = 0;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double[] currentAmps = new double[0];
  }

  public default void updateInputs(Inputs inputs) {}

  public default void setSpeed(double speed) {}

  public default void setVoltage(double voltage) {}

  public default double getVoltage() {
    return 0;
  }

  public default double getAngle() {
    return 0;
  }
}
