package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.states.ShooterState;
import frc.robot.util.Hub;
import frc.robot.util.Hub.Targets;

import java.util.function.Supplier;

import org.parts3492.partslib.PARTsUnit.PARTsUnitType;
import org.parts3492.partslib.command.PARTsCommandUtils;
import org.parts3492.partslib.command.PARTsSubsystem;

public abstract class Shooter extends PARTsSubsystem {
    private ShooterState shooterState = ShooterState.IDLE;

    private PIDController shooterPIDController;
    private SimpleMotorFeedforward shooterFeedforward;
    private Pose2d poseSupplier;

    public Shooter(Supplier <Pose2d> poseSupplier) {
        super("Shooter", RobotConstants.LOGGING);
        this.poseSupplier = poseSupplier.get();
        if (RobotConstants.DEBUGGING) {
            partsNT.putDouble("Shooter Speed", 0);
        }

        shooterPIDController = new PIDController(ShooterConstants.P, ShooterConstants.I, ShooterConstants.D);
        shooterFeedforward = new SimpleMotorFeedforward(ShooterConstants.S, ShooterConstants.V, ShooterConstants.A);

        shooterPIDController.setTolerance(ShooterConstants.PID_THRESHOLD);
    }

    // region Generic Subsystem Functions
    @Override
    public void outputTelemetry() {
        partsNT.putString("Shooter State", shooterState.toString());
        partsNT.putDouble("RPM", getRPM());
        partsNT.putDouble("Voltage", getVoltage());
        partsNT.putDouble("Get Setpoint", shooterPIDController.getSetpoint());
        partsNT.putBoolean("At Setpoint", shooterPIDController.atSetpoint());
        partsNT.putDouble("Current Error", shooterPIDController.getError());

        Targets zone = Hub.getZone(poseSupplier);
        partsNT.putString("Zone", zone == null ? "No zone" : zone.toString());
    }

    @Override
    public void stop() {
        shooterState = ShooterState.DISABLED;
    }

    @Override
    public void reset() {
        shooterState = ShooterState.IDLE;
    }

    @Override
    public void log() {
        partsLogger.logString("Shooter State", shooterState.toString());
    }

    @Override
    public void periodic() {
        if (RobotConstants.DEBUGGING) {
            setSpeed(partsNT.getDouble("Shooter Speed"));
        } else {
            switch (shooterState) {
                case CHARGING:
                case DISABLED:
                case IDLE:
                case SHOOTING:
                    double voltage = 0;
                    Targets zone = Hub.getZone(poseSupplier);
                    double zoneRPM = shooterState.getZoneRPM(zone);
                    double shooterRPM = shooterState.getRPM();
                    shooterPIDController.setSetpoint(shooterRPM);

                    double pidCalc = shooterPIDController.calculate(getRPM(), shooterRPM);
                    double ffCalc = shooterFeedforward.calculate((shooterPIDController.getSetpoint() * Math.PI
                            * ShooterConstants.SHOOTER_WHEEL_RADIUS.to(PARTsUnitType.Meter) * 2) / 60);

                    voltage = pidCalc + ffCalc;

                    setVoltage(voltage);
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
     * Sets the speed of the Shooter.
     * 
     * @param speed The speed between <code>-1.0</code> and <code>1.0</code>.
     */
    protected abstract void setSpeed(double speed);

    protected abstract void setVoltage(double voltage);

    protected abstract double getVoltage();

    protected abstract double getRPM();

    public ShooterState getState() {
        return shooterState;
    }

    public Command shoot() {
        return PARTsCommandUtils.setCommandName("Kicker.shoot", this.runOnce(() -> {
            shooterState = ShooterState.SHOOTING;
        }));
    }

    public Command idle() {
        return PARTsCommandUtils.setCommandName("Kicker.idle", this.runOnce(() -> {
            shooterState = ShooterState.IDLE;
        }));
    }
    // endregion
}
