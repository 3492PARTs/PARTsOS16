package frc.robot.subsystems.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.RobotConstants;
import frc.robot.states.ClimberState;
import frc.robot.util.PARTs.Classes.PARTsCommandUtils;
import frc.robot.states.ShooterState;
import frc.robot.util.PARTs.Classes.Abstracts.PARTsSubsystem;

public abstract class Climber extends PARTsSubsystem{
    private ClimberState ClimberState = ClimberState.IDLE;

    public Climber() {
        super("Climber");
        if (RobotConstants.DEBUGGING) {
            partsNT.putDouble("Climber Speed", 0);
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
            setSpeed(partsNT.getDouble("Climber Speed"));
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
