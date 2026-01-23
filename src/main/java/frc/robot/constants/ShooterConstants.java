package frc.robot.constants;

import frc.robot.util.PARTs.Classes.PARTsUnit;
import frc.robot.util.PARTs.Classes.PARTsUnit.PARTsUnitType;

public class ShooterConstants {
    public static final double SHOOTER_RPM = 3271; 
    public static final int LEFT_MOTOR_ID = 3;
    public static final int RIGHT_MOTOR_ID = 5;


    public static final PARTsUnit SHOOTER_WHEEL_RADIUS = new PARTsUnit(1.5, PARTsUnitType.Inch);

    // PID Controller
    public static final double P = 0.003;
    public static final double I = 0;
    public static final double D = 0;

    // Feedforward
    public static final double S = 0.27506;
    public static final double V = 0.53757;
    public static final double A = 0.057542;
}
