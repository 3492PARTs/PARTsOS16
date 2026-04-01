package frc.robot.util;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class PARTsThroughBoreEncoder {
  private final DutyCycleEncoder duty;
  private final double offsetDeg;

  /**
   * Create a new Through Bore Encoder.
   * 
   * @param dioPort   THe Digital IO port of the encoder.
   * @param offsetDeg The starting offset of the encoder in degrees.
   */
  public PARTsThroughBoreEncoder(int dioPort, double offsetDeg) {
    this.duty = new DutyCycleEncoder(new DigitalInput(dioPort)); //new DutyCycle(new DigitalInput(dioPort));
    this.offsetDeg = offsetDeg;
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
  public double getAngleDeg() {
    double angle = getAbsoluteAngleDeg() - offsetDeg;
    //angle %= 360.0;
    //if (angle < 0)
      //angle += 360.0;
    return angle;
  }
}
