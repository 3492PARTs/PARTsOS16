package frc.robot.subsystems.Kicker;

import org.parts3492.partslib.command.PARTsCommandUtils;
import org.parts3492.partslib.command.PARTsSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.constants.KickerConstants.KickerState;
import frc.robot.constants.RobotConstants;

public abstract class Kicker extends PARTsSubsystem {

    private KickerState kickerState = KickerState.IDLE;

    protected boolean debug = false;
    private Command toggleDebug = Commands.runOnce(() -> debug = !debug).ignoringDisable(true);

    public Kicker() {
        super("Kicker");
        if (RobotConstants.COMPETITION)
            debug = false;

        if (RobotContainer.debug || debug) {
            partsNT.putDouble("Kicker Speed", 0, true);
        }

        partsNT.putSmartDashboardSendable("Toggle Kicker Debug", toggleDebug, !RobotConstants.COMPETITION);
    }

    // region Generic Subsystem Functions
    @Override
    public void outputTelemetry() {
        partsNT.putString("Kicker State", kickerState.toString(), !RobotConstants.COMPETITION);
        partsNT.putBoolean("Kicker Debug Active", debug, !RobotConstants.COMPETITION);
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
            setSpeed(partsNT.getDouble("Kicker Speed", true));
        } else {
            switch (kickerState) {
                case ROLLING:
                case DISABLED:
                case IDLE:
                    setSpeed(kickerState.getSpeed());
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
    // endregion
}
