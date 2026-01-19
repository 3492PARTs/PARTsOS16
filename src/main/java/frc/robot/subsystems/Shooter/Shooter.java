package frc.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.RobotConstants;
import frc.robot.states.ShooterState;
import org.parts3492.partslib.command.PARTsCommandUtils;
import frc.robot.states.ShooterState;
import org.parts3492.partslib.command.PARTsSubsystem;

public abstract class Shooter extends PARTsSubsystem{
    private ShooterState shooterState = ShooterState.IDLE;

    public Shooter() {
        super("Shooter", RobotConstants.LOGGING);
        if (RobotConstants.DEBUGGING) {
            partsNT.putDouble("Shooter Speed", 0);
        }
    }

    //region Generic Subsystem Functions
    @Override
    public void outputTelemetry() {
    }

    @Override
    public void stop() {
    }

    @Override
    public void reset() {
    }

    @Override
    public void log() {
    }
    //endregion

    @Override
    public void periodic() {
        if (RobotConstants.DEBUGGING) {
            setSpeed(partsNT.getDouble("Shooter Speed"));
        }
        else {
            switch (shooterState) {
                case CHARGING:
                    setSpeed(shooterState.getSpeed());
                    break;
                case DISABLED:
                    setSpeed(shooterState.getSpeed());
                    break;
                case IDLE:
                    setSpeed(shooterState.getSpeed());
                    break;
                case SHOOTING:
                    setSpeed(shooterState.getSpeed());
                    break;
                default:
                    setSpeed(0);
                    break;

            }
        }
    }

    //region Custom Public Functions
    /** Sets the speed of the Shooter.
     * @param speed The speed between <code>-1.0</code> and <code>1.0</code>.
    */
    protected abstract void setSpeed(double speed);

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
