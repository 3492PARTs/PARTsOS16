package frc.robot.subsystems.Climber;

import org.parts3492.partslib.command.PARTsCommandUtils;
import org.parts3492.partslib.command.PARTsSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ClimberConstants;
import frc.robot.constants.ClimberConstants.ClimberState;

public abstract class Climber extends PARTsSubsystem {
    private ClimberConstants.ClimberState climberState = ClimberState.IDLE;

    public Climber() {
        super("Climber");
    }

    // region Generic Subsystem Functions
    @Override
    public void outputTelemetry() {
        partsNT.putString("Climber State", climberState.toString());
    }

    @Override
    public void stop() {
        climberState = climberState.DISABLED;
    }

    @Override
    public void reset() {
        climberState = climberState.IDLE;
    }

    @Override
    public void log() {
        partsLogger.logString("Climber State", climberState.toString());
    }
    // endregion

    @Override
    public void periodic() {
        switch (climberState) {
            case CLIMBING:
                setSpeed(climberState.getSpeed());
                break;
            case DECLIMB:
                setSpeed(climberState.getSpeed());
                break;
            case DISABLED:
                setSpeed(climberState.getSpeed());
                break;
            case IDLE:
                setSpeed(climberState.getSpeed());
                break;
            default:
                setSpeed(0);
                break;
        }
    }

    // region Custom Public Functions
    /**
     * Sets the speed of the Shooter.
     * 
     * @param speed The speed between <code>-1.0</code> and <code>1.0</code>.
     */
    protected abstract void setSpeed(double speed);

    public ClimberState getState() {
        return climberState;
    }

    public Command idle() {
        return PARTsCommandUtils.setCommandName("Command Idle", this.runOnce(() -> {
            climberState = ClimberState.IDLE;
        }));
    }

    public Command climb() {
        return PARTsCommandUtils.setCommandName("Command Climb", this.runOnce(() -> {
            climberState = ClimberConstants.ClimberState.CLIMBING;
        }));
    }

    public Command declimb() {
        return PARTsCommandUtils.setCommandName("Command Declimb", this.runOnce(() -> {
            climberState = ClimberConstants.ClimberState.DECLIMB;
        }));
    }
    // endregion
}
