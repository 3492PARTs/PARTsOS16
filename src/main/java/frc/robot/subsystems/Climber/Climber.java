package frc.robot.subsystems.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.states.ClimberState;
import frc.robot.util.PARTs.Classes.PARTsCommandUtils;
import frc.robot.util.PARTs.Classes.Abstracts.PARTsSubsystem;

public abstract class Climber extends PARTsSubsystem {
    private ClimberState climberState = ClimberState.IDLE;

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
            climberState = ClimberState.CLIMBING;
        }));
    }

    public Command declimb() {
        return PARTsCommandUtils.setCommandName("Command Declimb", this.runOnce(() -> {
            climberState = ClimberState.DECLIMB;
        }));
    }
    // endregion
}
