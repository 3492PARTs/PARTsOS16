package frc.robot.constants;

import org.parts3492.partslib.PARTsUnit;
import org.parts3492.partslib.PARTsUnit.PARTsUnitType;

public class ShooterConstants {
    public static final double SHOOTER_RPM = 3000; 
    public static final int LEFT_MOTOR_ID = 33;
    public static final int RIGHT_MOTOR_ID = 35;
    public static final String CAN_BUS_NAME = "bye";


    public static final PARTsUnit SHOOTER_WHEEL_RADIUS = new PARTsUnit(1.5, PARTsUnitType.Inch);
    // TODO: Get actual wheel weight.
    public static final PARTsUnit SHOOTER_WEEL_WEIGHT = new PARTsUnit(4, PARTsUnitType.Pound);

    // PID Controller
    public static final double P = 0.0005;
    public static final double I = 0;
    public static final double D = 0;
    public static final int PID_THRESHOLD = 50;

    // Feedforward
    public static final double S = 0.27506;
    public static final double V = 0.53757;
    public static final double A = 0.057542;
}
