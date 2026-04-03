package frc.robot.util;

import org.parts3492.partslib.PARTsUnit;
import org.parts3492.partslib.PARTsUnit.PARTsUnitType;
import org.parts3492.partslib.command.PARTsSubsystem;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.constants.TurretConstants;

public class PARTsThroughBoreEncoder  extends PARTsSubsystem{
  private final DutyCycleEncoder duty;
  private final double offsetDeg;
  private PARTsUnit accumulator = new PARTsUnit(0, PARTsUnitType.Angle);
  private PARTsUnit previousValue = new PARTsUnit(0, PARTsUnitType.Angle);

  /**+
   * Create a new Through Bore Encoder.
   * 
   * @param dioPort   THe Digital IO port of the encoder.
   * @param offsetDeg The starting offset of the encoder in degrees.
   */
  public PARTsThroughBoreEncoder(int dioPort, double offsetDeg) {
    this.duty = new DutyCycleEncoder(new DigitalInput(dioPort)); //new DutyCycle(new DigitalInput(dioPort));
    this.offsetDeg = offsetDeg;

    accumulator = getAngle();
    previousValue = accumulator;
  }

  /** Raw absolute angle in degrees [0, 360). */
  public double getAbsoluteAngleDeg() {
    // Fraction [0, 1). This is the PWM duty cycle.
    double d = duty.get();

    // Basic mapping; see note below about min/max duty range.
    double angle = d * 360.0;

    //angle %= 360.0;
    //if (angle < 0)
      //angle += 360.0;
    return angle;
  }

  /** Absolute angle with your mechanical zero applied, degrees [0, 360). */
  public PARTsUnit getAngle() {
    double angle = getAbsoluteAngleDeg() - offsetDeg;
    //angle %= 360.0;
    //if (angle < 0)
      //angle += 360.0;
    return new PARTsUnit(angle, PARTsUnitType.Angle);
  }

  @Override
  public void outputTelemetry() {
  }

  @Override
  public void stop() {
  }

  @Override
  public void reset() {
  }

  @Override
  public void log() {
  }

  @Override
  public void periodic() {
    PARTsUnit current = getAngle();
    double difference = MathUtil.inputModulus(current.getValue() - previousValue.getValue(), -180, 180);
    accumulator = new PARTsUnit(accumulator.getValue() + difference , PARTsUnitType.Angle);
    previousValue = current;
  }

  public PARTsUnit getAccumulatedAngle() {
    return accumulator;
  }
}
