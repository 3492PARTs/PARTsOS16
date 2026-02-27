package frc.robot.subsystems.Hopper;

import org.parts3492.partslib.command.PARTsCommandUtils;
import org.parts3492.partslib.command.PARTsSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.RobotConstants;
import frc.robot.states.HopperState;

public abstract class Hopper extends PARTsSubsystem {
    private HopperState hopperstate = HopperState.IDLE;

    public Hopper () {
        super("Hopper");

        if (RobotConstants.DEBUGGING) {
            partsNT.putDouble("Hopper Speed", 0);
        }
    }

    //region Generic Subsystem Functions
    @Override
    public void outputTelemetry() {
        partsNT.putString("Hopper State", hopperstate.toString());
    }

    @Override
    public void stop() {
        hopperstate = HopperState.DISABLED;
    }

    @Override
    public void reset() {
        hopperstate = HopperState.IDLE;
    }

    @Override
    public void log() {
        partsLogger.logString("Hopper State", hopperstate.toString());
    }

    @Override
    public void periodic() {
        if (RobotConstants.DEBUGGING) {
            setSpeed(partsNT.getDouble("Hopper Speed"));
        }
        else {
            switch(hopperstate) {
            case DISABLED:
            case ROLLING:
            case IDLE:
            case BACKROLLING:
                setSpeed(hopperstate.getSpeed());
                break;
            default:
                setSpeed(0);
                break;
        }
        }
    }
    //endregion

    //region Custom Public Functions
    /** Sets the speed of the Hopper.
     * @param speed The speed between <code>-1.0</code> and <code>1.0</code>.
    */
    protected abstract void setSpeed(double speed);

    public HopperState getState() { return hopperstate; }

    public Command roll() {
        return PARTsCommandUtils.setCommandName("Hopper.roll", this.runOnce(() -> {
            hopperstate = HopperState.ROLLING;
        }));
    }

    public Command backRoll() {
        return PARTsCommandUtils.setCommandName("Hopper.backRoll", this.runOnce(() -> {
            hopperstate = HopperState.BACKROLLING;
        }));
    }

    public Command idle() {
        return PARTsCommandUtils.setCommandName("Hopper.idle", this.runOnce(() -> {
            hopperstate = HopperState.IDLE;
        }));
    }
    //endregion
}
