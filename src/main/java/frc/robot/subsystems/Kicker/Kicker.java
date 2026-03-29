package frc.robot.subsystems.Kicker;

import org.parts3492.partslib.PARTsUnit.PARTsUnitType;
import org.parts3492.partslib.command.PARTsCommandUtils;
import org.parts3492.partslib.command.PARTsSubsystem;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.constants.KickerConstants.KickerState;
import frc.robot.constants.KickerConstants;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.ShooterConstants;

public abstract class Kicker extends PARTsSubsystem {

    private PIDController kickerPIDController;
    private SimpleMotorFeedforward kickerFeedforward;

    private KickerState kickerState = KickerState.IDLE;

    protected boolean debug = false;
    private Command toggleDebug = Commands.runOnce(() -> {
        debug = !debug;
        partsNT.putDouble("Kicker Speed", 0, true);
    }).ignoringDisable(true);

    public Kicker() {
        super("Kicker");
        if (RobotConstants.COMPETITION)
            debug = false;

        if (RobotContainer.debug || debug) {
            partsNT.putDouble("Kicker Speed", 0, true);
        }

        kickerPIDController = new PIDController(KickerConstants.P, KickerConstants.I, KickerConstants.D);
        kickerFeedforward = new SimpleMotorFeedforward(KickerConstants.S, KickerConstants.V, KickerConstants.A);
        kickerPIDController.setTolerance(KickerConstants.PID_THRESHOLD);

        partsNT.putSmartDashboardSendable("Toggle Kicker Debug", toggleDebug, !RobotConstants.COMPETITION);
    }

    // region Generic Subsystem Functions
    @Override
    public void outputTelemetry() {
        partsNT.putString("Kicker State", kickerState.toString(), !RobotConstants.COMPETITION);
        partsNT.putBoolean("Kicker Debug Active", debug, !RobotConstants.COMPETITION);
        partsNT.putDouble("Kicker RPM", getRPM(), !RobotConstants.COMPETITION);
    }

    @Override
    public void stop() {
        kickerState = KickerState.DISABLED;
    }

    @Override
    public void reset() {
        kickerState = KickerState.IDLE;
    }

    @Override
    public void log() {
        partsLogger.logString("Kicker State", kickerState.toString(), RobotContainer.debug || debug);
    }

    @Override
    public void periodic() {
        if (RobotContainer.debug || debug) {
            double rpm = partsNT.getDouble("Kicker Speed", true);
            setVoltage(calculateRPMVoltage(rpm));
        } else {
            switch (kickerState) {
                case DISABLED:
                case IDLE:
                    setSpeed(0);
                    break;
                case ROLLING:
                    setVoltage(calculateRPMVoltage(kickerState.getRPM()));
                    break;
                default:
                    setSpeed(0);
                    break;
            }
        }
    }
    // endregion

    // region Custom Public Functions
    /**
     * Sets the speed of the Kicker.
     * 
     * @param speed The speed between <code>-1.0</code> and <code>1.0</code>.
     */
    protected abstract void setSpeed(double speed);

    protected abstract double getRPM();

    protected abstract void setVoltage(double voltage);

    protected abstract double getVoltage();

    public KickerState getState() {
        return kickerState;
    }

    public Command roll() {
        return PARTsCommandUtils.setCommandName("Kicker.roll", this.runOnce(() -> {
            kickerState = KickerState.ROLLING;
        }));
    }

    public Command idle() {
        return PARTsCommandUtils.setCommandName("Kicker.idle", this.runOnce(() -> {
            kickerState = KickerState.IDLE;
        }));
    }
    
    public boolean withinSetpointRange() {
        return Math.abs(kickerPIDController.getSetpoint() - getRPM()) < 300;
    }

    private double calculateRPMVoltage(double rpm) {
        kickerPIDController.setSetpoint(rpm);
        double pidCalc = kickerPIDController.calculate(getRPM(), rpm);
        double ffCalc = kickerFeedforward.calculate((kickerPIDController.getSetpoint() * Math.PI
                * ShooterConstants.SHOOTER_WHEEL_RADIUS.to(PARTsUnitType.Meter) * 2) / 60);

        double voltage = pidCalc + ffCalc;

        return voltage;
    }
    // endregion
}
