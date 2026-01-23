package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.states.ShooterState;
import frc.robot.util.PARTs.Classes.PARTsCommandUtils;
import frc.robot.states.ShooterState;
import frc.robot.util.PARTs.Classes.Abstracts.PARTsSubsystem;
import frc.robot.util.PARTs.Classes.PARTsUnit.PARTsUnitType;

public abstract class Shooter extends PARTsSubsystem{
    private ShooterState shooterState = ShooterState.IDLE;

    private PIDController shooterPIDController;
    private SimpleMotorFeedforward shooterFeedforward;

    public Shooter() {
        super("Shooter");
        if (RobotConstants.DEBUGGING) {
            partsNT.putDouble("Shooter Speed", 0);
        }

        shooterPIDController = new PIDController(ShooterConstants.P, ShooterConstants.I, ShooterConstants.D);
        shooterFeedforward = new SimpleMotorFeedforward(ShooterConstants.S, ShooterConstants.V, ShooterConstants.A);

        shooterPIDController.setTolerance(100);
    }

    //region Generic Subsystem Functions
    @Override
    public void outputTelemetry() {
        partsNT.putString("Shooter State", shooterState.toString());
        partsNT.putDouble("RPM", getRPM());
        partsNT.putDouble("Voltage", getVoltage());
        partsNT.putDouble("Get Setpoint", shooterPIDController.getSetpoint());
        partsNT.putBoolean("At Setpoint", shooterPIDController.atSetpoint());
        partsNT.putDouble("Current Error", shooterPIDController.getError());
    }

    @Override
    public void stop() {
        shooterState = ShooterState.DISABLED;
    }

    @Override
    public void reset() {
        shooterState = ShooterState.IDLE;
    }

    @Override
    public void log() {
        partsLogger.logString("Shooter State", shooterState.toString());
    }
    //endregion



    @Override
    public void periodic() {
        if (RobotConstants.DEBUGGING) {
            setSpeed(partsNT.getDouble("Shooter Speed"));
        }
        else {
            double voltage = 0;
            shooterPIDController.setSetpoint(shooterState.getRPM());

            double pidCalc = shooterPIDController.calculate(getRPM(), shooterState.getRPM());
            double ffCalc = shooterFeedforward.calculate((shooterPIDController.getSetpoint() * Math.PI * ShooterConstants.SHOOTER_WHEEL_RADIUS.to(PARTsUnitType.Inch) * 2) / (60 * 40));

            voltage = (pidCalc * 0) + ffCalc;

            setVoltage(voltage);
        }
    }

    //region Custom Public Functions
    /** Sets the speed of the Shooter.
     * @param speed The speed between <code>-1.0</code> and <code>1.0</code>.
    */
    protected abstract void setSpeed(double speed);

    protected abstract void setVoltage(double voltage);

    protected abstract double getVoltage();

    protected abstract double getRPM();

    public ShooterState getState() { return shooterState; }

    public Command shoot() {
        return PARTsCommandUtils.setCommandName("Command Shoot", this.runOnce(() -> {
            shooterState = ShooterState.SHOOTING;
        }));
    }

    public Command idle() {
        return PARTsCommandUtils.setCommandName("Command Idle", this.runOnce(() -> {
            shooterState = ShooterState.IDLE;
        }));
    }
    //endregion
}
