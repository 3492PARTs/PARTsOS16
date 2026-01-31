package frc.robot.subsystems.Hopper;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.states.HopperState;
import frc.robot.util.PARTs.Classes.PARTsCommandUtils;
import frc.robot.util.PARTs.Classes.Abstracts.PARTsSubsystem;

public abstract class Hopper extends PARTsSubsystem {
    private HopperState hopperstate = HopperState.IDLE;

    public Hopper () {
        super("Hopper");
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
        switch(hopperstate) {
            case DISABLED:
                setSpeed(hopperstate.getSpeed());
                break;
            case ROLLING:
                setSpeed(hopperstate.getSpeed());
                break;
            case IDLE:
                setSpeed(hopperstate.getSpeed());
                break;
            case BACKROLLING:
                setSpeed(hopperstate.getSpeed());
                break;
            default:
                setSpeed(0);
                break;
            
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
        return PARTsCommandUtils.setCommandName("Command Roll", this.runOnce(() -> {
            hopperstate = HopperState.ROLLING;
        }));
    }

    public Command backRoll() {
        return PARTsCommandUtils.setCommandName("Command BackRoll", this.runOnce(() -> {
            hopperstate = HopperState.BACKROLLING;
        }));
    }

    public Command idle() {
        return PARTsCommandUtils.setCommandName("Command Hopper Idle", this.runOnce(() -> {
            hopperstate = HopperState.IDLE;
        }));
    }
    //endregion
}
