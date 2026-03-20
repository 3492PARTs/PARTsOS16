package frc.robot.constants;

import org.parts3492.partslib.PARTsUnit;
import org.parts3492.partslib.PARTsUnit.PARTsUnitType;

public class IntakeConstants {
    public enum IntakeState {
        IDLE(0, new PARTsUnit(0, PARTsUnitType.Angle)),
        DISABLED(0, new PARTsUnit(0, PARTsUnitType.Angle)),
        INTAKING(1, new PARTsUnit(194, PARTsUnitType.Angle)),
        SHOOTING(0, new PARTsUnit(45, PARTsUnitType.Angle)),
        HOME(0, new PARTsUnit(0, PARTsUnitType.Angle)),
        MANUALPIVOT(0, new PARTsUnit(0, PARTsUnitType.Angle)),
        HOLD(0, new PARTsUnit(0, PARTsUnitType.Angle));

        private double speed;
        private PARTsUnit angle;

        private IntakeState(double speed, PARTsUnit angle) {
            this.speed = speed;
            this.angle = angle;
        }

        public double getSpeed() {
            return speed;
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

    // PID Controller
    public static final double P = 0.18;
    public static final double I = 0;
    public static final double D = 0;
    public static final int PID_THRESHOLD = 3;

    public static final double INTAKE_MAX_VELOCITY = 800;
    public static final double INTAKE_MAX_ACCELERATION = 1000;
}
