package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.ShooterConstants.ShooterState;
import frc.robot.util.Hub;
import frc.robot.util.Trench;
import frc.robot.util.Hub.Targets;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.parts3492.partslib.PARTsUnit.PARTsUnitType;
import org.parts3492.partslib.command.PARTsCommandUtils;
import org.parts3492.partslib.command.PARTsSubsystem;

public abstract class Shooter extends PARTsSubsystem {
    private ShooterState shooterState = ShooterState.IDLE;

    private PIDController shooterPIDController;
    private SimpleMotorFeedforward shooterFeedforward;
    private Supplier<Pose2d> poseSupplier;

    protected boolean debug = false;
    private Command toggleDebug = Commands.runOnce(()-> debug = !debug).ignoringDisable(true);

    /**
     * Creates a new Shooter subsystem.
     * @param poseSupplier The supplier for the robot's pose, used to get the distance to the hub and trench.
     */
    public Shooter(Supplier<Pose2d> poseSupplier) {
        super("Shooter", RobotConstants.LOGGING);
        if (RobotConstants.COMPETITION) debug = false;
        
        this.poseSupplier = poseSupplier;
        if (RobotContainer.debug || debug) {
            partsNT.putDouble("Shooter Speed", 0, true);
        }

        shooterPIDController = new PIDController(ShooterConstants.P, ShooterConstants.I, ShooterConstants.D);
        shooterFeedforward = new SimpleMotorFeedforward(ShooterConstants.S, ShooterConstants.V, ShooterConstants.A);

        shooterPIDController.setTolerance(ShooterConstants.PID_THRESHOLD);

        partsNT.putSmartDashboardSendable("Toggle Shooter Debug", toggleDebug, !RobotConstants.COMPETITION);
    }

    // region Generic Subsystem Functions
    @Override
    public void outputTelemetry() {
        partsNT.putString("Shooter State", shooterState.toString(), RobotContainer.debug || debug);
        partsNT.putDouble("RPM", getRPM(), true);
        partsNT.putDouble("Voltage", getVoltage(), RobotContainer.debug || debug);
        partsNT.putDouble("Get Setpoint", shooterPIDController.getSetpoint(), RobotContainer.debug || debug);
        partsNT.putBoolean("At Setpoint", shooterPIDController.atSetpoint(), true);
        partsNT.putDouble("Current Error", shooterPIDController.getError(), RobotContainer.debug || debug);
        partsNT.putBoolean("Shooter Debug Active", debug, !RobotConstants.COMPETITION);
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
        partsLogger.logString("Shooter State", shooterState.toString(), !RobotConstants.COMPETITION);
    }

    @Override
    public void periodic() {
        if (RobotContainer.debug || debug) {
            setSpeed(partsNT.getDouble("Shooter Speed", true));
        } else {
            Targets zone = Hub.getZone(poseSupplier.get());
             double shooterRPM = (shooterState == ShooterState.MANUAL) ? shooterState.getRPM() : ShooterState.getRPMFromDistanceToHub(poseSupplier.get());

            boolean inTrench = Trench.isUnderTrench(poseSupplier.get());
            if (inTrench) {
                shooterRPM = ShooterState.getZoneRPM(Targets.ZONE2);
            }

            partsNT.putString("Zone", inTrench ? "Trench" : zone == null ? "No zone" : zone.toString(), true);
            
            switch (shooterState) {
                case CHARGING:
                case DISABLED:
                case IDLE:
                    setSpeed(0);
                    break;
                case SHOOTING:
                case MANUAL:
                    double voltage = 0;
                    // double shooterRPM = shooterState.getRPM();
                    if (debug) {
                        shooterRPM = partsNT.getDouble("Shooter Speed", true);
                    }

                    shooterPIDController.setSetpoint(shooterRPM);
                    partsNT.putBoolean("In Setpoint Range", withinSetpointRange(), true);
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

    /**
     * Sets the voltage of the Shooter.
     * 
     * @param voltage The voltage between <code>-12.0</code> and <code>12.0</code>.
     */
    protected abstract void setVoltage(double voltage);

    /**
     * Gets the voltage of the Shooter.
     * 
     * @return The voltage between <code>-12.0</code> and <code>12.0</code>.
     */
    protected abstract double getVoltage();

    /**
     * Gets the RPM of the Shooter.
     * 
     * @return The RPM of the Shooter.
     */
    protected abstract double getRPM();

    /**
     * Gets the current state of the Shooter.
     * @return The current state of the Shooter.
     */
    public ShooterState getState() {
        return shooterState;
    }

    /**
     * Command to set the Shooter to the {@link ShooterState#SHOOTING SHOOTING} state.
     * @return The command.
     */
    public Command shoot() {
        return PARTsCommandUtils.setCommandName("Shooter.shoot", this.runOnce(() -> {
            shooterState = ShooterState.SHOOTING;
        }));
    }

    /**
     * Command to set the Shooter to the {@link ShooterState#IDLE IDLE} state.
     * @return The command.
     */
    public Command idle() {
        return PARTsCommandUtils.setCommandName("Shooter.idle", this.runOnce(() -> {
            shooterState = ShooterState.IDLE;
        }));
    }

    /**
     * Command to set the Shooter to the {@link ShooterState#MANUAL MANUAL} state.<p>
     * This allows manual control of the shooter RPM.
     * @return The command.
     */
    public Command manualShoot() {
        return PARTsCommandUtils.setCommandName("Shooter.manualShoot", this.runOnce(() -> {
            shooterState = ShooterState.MANUAL;
        }));
    }

    /**
     * Gets whether the Shooter is at the setpoint RPM.
     * @return A boolean supplier that returns true if the Shooter is at the setpoint RPM.
     */
    public BooleanSupplier atSetpoint() {
        return () -> shooterPIDController.atSetpoint();
    }

    /**
     * Gets the current setpoint of the Shooter.
     * @return A double supplier that returns the current setpoint of the Shooter in RPM.
     */
    public DoubleSupplier getSetpoint() {
        return () -> shooterPIDController.getSetpoint();
    }

    /**
     * Gets whether the Shooter is within a certain range of the setpoint RPM.
     * @return Boolean that returns true if the Shooter is within the setpoint range.
     */
    public boolean withinSetpointRange() {
        return Math.abs(shooterPIDController.getSetpoint() - getRPM()) < 500;
    }
    // endregion
}
