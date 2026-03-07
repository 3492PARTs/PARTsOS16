package frc.robot.constants;

import org.parts3492.partslib.PARTsUnit;
import org.parts3492.partslib.PARTsUnit.PARTsUnitType;

public class IntakeConstants {
    public enum IntakeState {
        IDLE(0, new PARTsUnit(0, PARTsUnitType.Angle)),
        DISABLED(0, new PARTsUnit(0, PARTsUnitType.Angle)),
        INTAKING(.9, new PARTsUnit(190, PARTsUnitType.Angle)),
        OUTTAKING(-.9, new PARTsUnit(190, PARTsUnitType.Angle)),
        SHOOTING(0.5, new PARTsUnit(45, PARTsUnitType.Angle)),
        TRAVELING(0, new PARTsUnit(160, PARTsUnitType.Angle)),
        HOME(0, new PARTsUnit(0, PARTsUnitType.Angle)),
        MANUALPIVOT(0, new PARTsUnit(0, PARTsUnitType.Angle));

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
    public static final double P = 0.07;
    public static final double I = 0;
    public static final double D = 0;
    public static final int PID_THRESHOLD = 1;

    public static final double INTAKE_MAX_VELOCITY = 800;
    public static final double INTAKE_MAX_ACCELERATION = 1000;

    // Feedforward
    public static final double S = 1.2398;
    public static final double V = 1.1537;
    public static final double A = 0.27146;
}
