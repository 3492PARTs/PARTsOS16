package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.ShooterConstants.ShooterState;
import frc.robot.constants.TurretConstants.TurretState;
import frc.robot.subsystems.Drivetrain.PARTsDrivetrain;
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
    private Supplier<Pose2d> robotPoseSupplier;
    private Supplier<TurretState> turretStateSupplier;
    private PARTsDrivetrain drivetrain;

    protected boolean debug = false;
    private Command toggleDebug = Commands.runOnce(() -> debug = !debug).ignoringDisable(true);

    public Shooter(Supplier<Pose2d> poseSupplier, PARTsDrivetrain drivetrain,
            Supplier<TurretState> turretSupplierState) {
        super("Shooter", RobotConstants.LOGGING);
        if (RobotConstants.COMPETITION)
            debug = false;

        this.robotPoseSupplier = poseSupplier;
        this.turretStateSupplier = turretSupplierState;
        this.drivetrain = drivetrain;

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
        partsNT.putString("Shooter State", shooterState.toString(), !RobotConstants.COMPETITION);
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
        }

        else {
            Targets zone = Hub.getZone(robotPoseSupplier.get());
            double timeOfFlight = (zone == null) ? 0 : zone.getTimeOfFlight();
            Targets calculatedZone = Hub.getZone(
                    robotPoseSupplier.get().plus(new Transform2d(drivetrain.getXVelocity().getValue() * timeOfFlight,
                            drivetrain.getYVelocity().getValue() * timeOfFlight, new Rotation2d())));

            if (zone == null && turretStateSupplier.get() == TurretState.TRACKING_CORNER) {
                calculatedZone = Targets.BEHIND_HUB;
            }

            double shooterRPM = (shooterState == ShooterState.MANUAL) ? shooterState.getRPM()
                    : ShooterState.getZoneRPM(calculatedZone);

            boolean inTrench = Trench.isUnderTrench(robotPoseSupplier.get());

            if (inTrench) {
                shooterRPM = ShooterState.getZoneRPM(Targets.TRENCH);
            }

            partsNT.putString("Zone", inTrench ? "Trench" : zone == null ? "No zone" : zone.toString(), true);

            switch (shooterState) {
                case DISABLED:
                case IDLE:
                    setSpeed(0);
                    break;
                case SHOOTING:
                case MANUAL:
                    double voltage = 0;
                    
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

    protected abstract void setVoltage(double voltage);

    protected abstract double getVoltage();

    protected abstract double getRPM();

    public ShooterState getState() {
        return shooterState;
    }

    public Command shoot() {
        return PARTsCommandUtils.setCommandName("Shooter.shoot", this.runOnce(() -> {
            shooterState = ShooterState.SHOOTING;
        }));
    }

    public Command idle() {
        return PARTsCommandUtils.setCommandName("Shooter.idle", this.runOnce(() -> {
            shooterState = ShooterState.IDLE;
        }));
    }

    public Command manualShoot() {
        return PARTsCommandUtils.setCommandName("Shooter.manualShoot", this.runOnce(() -> {
            shooterState = ShooterState.MANUAL;
        }));
    }

    public BooleanSupplier atSetpoint() {
        return () -> shooterPIDController.atSetpoint();
    }

    public DoubleSupplier getSetpoint() {
        return () -> shooterPIDController.getSetpoint();
    }

    public boolean withinSetpointRange() {
        return Math.abs(shooterPIDController.getSetpoint() - getRPM()) < 500;
    }
    // endregion
}
