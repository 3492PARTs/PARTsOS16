package frc.robot.subsystems.Kicker;

import org.parts3492.partslib.command.PARTsCommandUtils;
import org.parts3492.partslib.command.PARTsSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.RobotConstants;
import frc.robot.states.KickerState;

public abstract class Kicker extends PARTsSubsystem {

    private KickerState kickerState = KickerState.IDLE;

    public Kicker() {
        super("Kicker");

        if (RobotConstants.DEBUGGING) {
            partsNT.putDouble("Kicker Speed", 0);
        }
    }

    // region Generic Subsystem Functions
    @Override
    public void outputTelemetry() {
        partsNT.putString("Kicker State", kickerState.toString());
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
        partsLogger.logString("Kicker State", kickerState.toString());
    }

    @Override
    public void periodic() {
        if (RobotConstants.DEBUGGING) {
            setSpeed(partsNT.getDouble("Kicker Speed"));
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
    //endregion

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
        return PARTsCommandUtils.setCommandName("Command Roll", this.runOnce(() -> {
            kickerState = KickerState.ROLLING;
        }));
    }

    public Command idle() {
        return PARTsCommandUtils.setCommandName("Command Idle", this.runOnce(() -> {
            kickerState = KickerState.IDLE;
        }));
    }
    //endregion
}
