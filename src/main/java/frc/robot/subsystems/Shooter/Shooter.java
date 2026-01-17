package frc.robot.subsystems.Shooter;

import frc.robot.states.ShooterState;
import frc.robot.util.PARTs.Classes.Abstracts.PARTsSubsystem;

public abstract class Shooter extends PARTsSubsystem{
    private ShooterState shooterState = ShooterState.IDLE;

    @Override
    public void outputTelemetry() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'outputTelemetry'");
    }

    @Override
    public void stop() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'stop'");
    }

    @Override
    public void reset() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'reset'");
    }

    @Override
    public void log() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'log'");
    }

    @Override
    public void periodic() {
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

    public abstract void setSpeed(double speed);

    public ShooterState getState() {return shooterState; }
}
