package frc.robot.constants;

import org.parts3492.partslib.PARTsUnit;
import org.parts3492.partslib.PARTsUnit.PARTsUnitType;

public class IntakeConstants {
    public enum IntakeState {
        IDLE(0, new PARTsUnit(0, PARTsUnitType.Angle)),
        DISABLED(0, new PARTsUnit(0, PARTsUnitType.Angle)),
        INTAKING(5500, new PARTsUnit(193, PARTsUnitType.Angle)),
        REVERSE(-5500, new PARTsUnit(193, PARTsUnitType.Angle)),
        SHOOTING(2000, new PARTsUnit(80, PARTsUnitType.Angle)),
        HOME(0, new PARTsUnit(0, PARTsUnitType.Angle)),
        MANUALPIVOT(0, new PARTsUnit(0, PARTsUnitType.Angle)),
        HOLD(0, new PARTsUnit(0, PARTsUnitType.Angle));

        private double RPM;
        private PARTsUnit angle;

        private IntakeState(double speed, PARTsUnit angle) {
            this.RPM = speed;
            this.angle = angle;
        }

        public double getRPM() {
            return RPM;
        }

        public PARTsUnit getAngle() {
            return angle;
        }

        public void setAngle(PARTsUnit angle) {
            this.angle = angle;
        }
    }

    public static final int INTAKE_MOTOR_ID = 36;
    public static final int PIVOT_MOTOR_ID = 38;
    public static final String CAN_BUS_NAME = "bye";

    /** The pivot gear ratio. The total is {@code 36/1}. */
    public static final double PIVOT_GEAR_RATIO = (12.0 / 1.0) * (3.0 / 1.0);

    public static final double INTAKE_GEAR_RATIO = (2/1); 

    // PID Controller
    public static final double PIVOT_P = 0.18;
    public static final double PIVOT_I = 0;
    public static final double PIVOT_D = 0;
    public static final int PIVOT_PID_THRESHOLD = 3;

    public static final double INTAKE_P = 0.0005;
    public static final double INTAKE_I = 0;
    public static final double INTAKE_D = 0;
    public static final int INTAKE_PID_THRESHOLD = 25;

    public static final double INTAKE_MAX_VELOCITY = 800;
    public static final double INTAKE_MAX_ACCELERATION = 1000;

    // Intake Feedforward (Not Pivot)
    public static final double IntakeS = 0.1077;
    public static final double IntakeV = 1.15; // 1.1542
    public static final double IntakeA = 0.016109;
}
