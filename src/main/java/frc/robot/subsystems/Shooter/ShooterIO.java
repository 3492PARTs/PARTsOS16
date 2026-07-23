package frc.robot.subsystems.Shooter;

import org.littletonrobotics.junction.AutoLog;
import org.parts3492.partslib.PARTsUnit;

public interface ShooterIO {
  @AutoLog
  public static class ShooterInputs {
    public boolean leftConnected = false;
    public boolean rightConnected = false;
    public double RPM = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double[] currentAmps = new double[0];
  }

  public default void updateInputs(ShooterInputs inputs) {}

  public default void setRPM(double RPM) {}

  public default void setVoltage(double volts) {}

  public default void stop() {}

  public default void setSpeed(double speed) {}

  public default double getVoltage() {
    return 0;
  }

  public default double getRPM() {
    return 0;
  }

  public default PARTsUnit getLinearPosition() {
    return null;
  }
}
