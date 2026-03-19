package frc.robot.subsystems.Hopper;

import org.parts3492.partslib.command.PARTsCommandUtils;
import org.parts3492.partslib.command.PARTsSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.constants.HopperConstants.HopperState;
import frc.robot.constants.RobotConstants;

public abstract class Hopper extends PARTsSubsystem {
    private HopperState hopperstate = HopperState.IDLE;

    protected boolean debug = false;
    private Command toggleDebug = Commands.runOnce(() -> debug = !debug).ignoringDisable(true);

    public Hopper() {
        super("Hopper");
        if (RobotConstants.COMPETITION)
            debug = false;

        if (RobotContainer.debug || debug) {
            partsNT.putDouble("Hopper Speed", 0, true);
        }

        partsNT.putSmartDashboardSendable("Toggle Hopper Debug", toggleDebug, !RobotConstants.COMPETITION);
    }

    // region Generic Subsystem Functions
    @Override
    public void outputTelemetry() {
        partsNT.putString("Hopper State", hopperstate.toString(), !RobotConstants.COMPETITION);
        partsNT.putBoolean("Hopper Debug Active", debug, !RobotConstants.COMPETITION);
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
        partsLogger.logString("Hopper State", hopperstate.toString(), RobotContainer.debug || debug);
    }

    @Override
    public void periodic() {
        if (RobotContainer.debug || debug) {
            setSpeed(partsNT.getDouble("Hopper Speed", true));
        } else {
            switch (hopperstate) {
                case DISABLED:
                case ROLLING:
                case IDLE:
                case REVERSE:
                    setSpeed(hopperstate.getSpeed());
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

    public HopperState getState() {
        return hopperstate;
    }

    public Command roll() {
        return PARTsCommandUtils.setCommandName("Hopper.roll", Commands.runOnce(() -> {
            hopperstate = HopperState.ROLLING;
        }));
    }

    public Command idle() {
        return PARTsCommandUtils.setCommandName("Hopper.idle", Commands.runOnce(() -> {
            hopperstate = HopperState.IDLE;
        }));
    }

    public Command reverse() {
        return PARTsCommandUtils.setCommandName("Hopper.reverse", Commands.runOnce(() -> {
            hopperstate = HopperState.REVERSE;
        }));
    }
    // endregion
}
