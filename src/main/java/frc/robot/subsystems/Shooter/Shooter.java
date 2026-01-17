package frc.robot.subsystems.Shooter;

import frc.robot.constants.RobotConstants;
import frc.robot.states.ShooterState;
import frc.robot.util.PARTs.Classes.Abstracts.PARTsSubsystem;

public abstract class Shooter extends PARTsSubsystem{
    private ShooterState shooterState = ShooterState.IDLE;

    public Shooter() {
        super("Shooter");
        if (RobotConstants.DEBUGGING) {
            partsNT.putDouble("Shooter Speed", 0);
        }
    }

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

    public abstract void setSpeed(double speed);

    public ShooterState getState() {return shooterState; }
}
