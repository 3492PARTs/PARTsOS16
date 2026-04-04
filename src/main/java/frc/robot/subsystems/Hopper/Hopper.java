package frc.robot.subsystems.Hopper;

import org.parts3492.partslib.PARTsUnit.PARTsUnitType;
import org.parts3492.partslib.command.PARTsCommandUtils;
import org.parts3492.partslib.command.PARTsSubsystem;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.constants.HopperConstants.HopperState;
import frc.robot.constants.KickerConstants;
import frc.robot.constants.HopperConstants;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.ShooterConstants;

public abstract class Hopper extends PARTsSubsystem {
    private PIDController hopperPIDController;
    private HopperState hopperState = HopperState.IDLE;
    private SimpleMotorFeedforward hopperFeedForward;

    protected boolean debug = false;
    private Command toggleDebug = Commands.runOnce(() -> {
        debug = !debug;
        partsNT.putDouble("Hopper Speed", 0, true);
    }).ignoringDisable(true);

    private Timer timer = new Timer();

    public Hopper() {
        super("Hopper");
        if (RobotConstants.COMPETITION)
            debug = false;

        if (RobotContainer.debug || debug) {
            partsNT.putDouble("Set Hopper RPM", 0, true);
        }
        hopperPIDController = new PIDController(HopperConstants.P, HopperConstants.I, HopperConstants.D);
        hopperFeedForward = new SimpleMotorFeedforward(HopperConstants.S, HopperConstants.V, HopperConstants.A);
        hopperPIDController.setTolerance(HopperConstants.PID_THRESHOLD);

        partsNT.putSmartDashboardSendable("Toggle Hopper Debug", toggleDebug, !RobotConstants.COMPETITION);
    }

    // region Generic Subsystem Functions
    @Override
    public void outputTelemetry() {
        partsNT.putString("Hopper State", hopperState.toString(), !RobotConstants.COMPETITION);
        partsNT.putBoolean("Hopper Debug Active", debug, !RobotConstants.COMPETITION);
        partsNT.putDouble("Hopper RPM", getRPM(), !RobotConstants.COMPETITION);
    }

    @Override
    public void stop() {
        hopperState = HopperState.DISABLED;
    }

    @Override
    public void reset() {
        hopperState = HopperState.IDLE;
    }

    @Override
    public void log() {
        partsLogger.logString("Hopper State", hopperState.toString(), RobotContainer.debug || debug);
    }

    @Override
    public void periodic() {
        if (RobotContainer.debug || debug) {
            setVoltage(calculateRPMVoltage(partsNT.getDouble("Set Hopper RPM", true)));
        } else {
            switch (hopperState) {
                case DISABLED:
                case IDLE:
                    setSpeed(hopperState.getSpeed());
                    break;
                case ROLLING:
                case REVERSE:
                    setVoltage(calculateRPMVoltage(hopperState.getRPM()));

                    /*if (timer.get() > 1.5 && hopperState == HopperState.ROLLING) {
                        timer.restart();
                        hopperState = HopperState.REVERSE;
                    }

                    if (timer.get() > .2 && hopperState == HopperState.REVERSE) {
                        timer.restart();
                        hopperState = HopperState.ROLLING;
                    }*/

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
     * Sets the speed of the Hopper.
     * 
     * @param speed The speed between <code>-1.0</code> and <code>1.0</code>.
     */
    protected abstract void setSpeed(double speed);

    protected abstract double getRPM();

    protected abstract void setVoltage(double voltage);

    protected abstract double getVoltage();

    public HopperState getState() {
        return hopperState;
    }

    public Command roll() {
        return PARTsCommandUtils.setCommandName("Hopper.roll", Commands.runOnce(() -> {
            hopperState = HopperState.ROLLING;
            timer.restart();
        }));
    }

    public Command idle() {
        return PARTsCommandUtils.setCommandName("Hopper.idle", Commands.runOnce(() -> {
            hopperState = HopperState.IDLE;
        }));
    }

    public Command reverse() {
        return PARTsCommandUtils.setCommandName("Hopper.reverse", Commands.runOnce(() -> {
            hopperState = HopperState.REVERSE;
            timer.restart();
        }));
    }

    private double calculateRPMVoltage(double rpm) {
        hopperPIDController.setSetpoint(rpm);
        double pidCalc = hopperPIDController.calculate(getRPM(), rpm);
        double ffCalc = hopperFeedForward.calculate((hopperPIDController.getSetpoint() * Math.PI
                * HopperConstants.HOPPER_ROLLER_RADIUS.to(PARTsUnitType.Meter) * 2) / 60);

        double voltage = pidCalc + ffCalc;

        return voltage;
    }
    // endregion
}
