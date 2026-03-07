package frc.robot.subsystems.Turret;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.TurretConstants;
import frc.robot.constants.TurretConstants.TurretState;
import frc.robot.util.Field;
import frc.robot.util.Hub;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.parts3492.partslib.command.PARTsCommandUtils;
import org.parts3492.partslib.command.PARTsSubsystem;

public abstract class Turret extends PARTsSubsystem {
    private TurretState turretState = TurretState.IDLE;

    private ProfiledPIDController turretPIDController;
    private SimpleMotorFeedforward turretFeedforward;
    private Supplier<Pose2d> robotPoseSupplier;

    protected boolean debug = false;
    private Command toggleDebug = Commands.runOnce(()-> debug = !debug).ignoringDisable(true);

    public Turret(Supplier<Pose2d> robotPoseSupplier) {
        super("Turret", RobotConstants.LOGGING);
        if (RobotConstants.COMPETITION) debug = false;

        if (RobotContainer.debug || debug) {
         partsNT.putDouble("Turret Speed", 0, true);
         partsNT.putDouble("Turret Angle", 0, true);
        }

        this.robotPoseSupplier = robotPoseSupplier;

        turretPIDController = new ProfiledPIDController(TurretConstants.P, TurretConstants.I, TurretConstants.D, new TrapezoidProfile.Constraints(TurretConstants.TURRET_MAX_VELOCITY, TurretConstants.TURRET_MAX_ACCELERATION));
        turretFeedforward = new SimpleMotorFeedforward(TurretConstants.S, TurretConstants.V, TurretConstants.A);

        turretPIDController.setTolerance(TurretConstants.PID_THRESHOLD);

        partsNT.putSmartDashboardSendable("Toggle Turret Debug", toggleDebug, !RobotConstants.COMPETITION);
    }

    // region Generic Subsystem Functions
    @Override
    public void outputTelemetry() {
        partsNT.putString("Turret State", turretState.toString(), !RobotConstants.COMPETITION);
        partsNT.putDouble("Angle", getAngle(), true);
        partsNT.putDouble("Voltage", getVoltage(), RobotContainer.debug || debug);
        partsNT.putDouble("Get Setpoint", turretPIDController.getSetpoint().position, RobotContainer.debug || debug);
        partsNT.putBoolean("At Setpoint", turretPIDController.atSetpoint(), true);
        partsNT.putDouble("Current Error", turretPIDController.getPositionError(), RobotContainer.debug || debug);
        partsNT.putDouble("Get Angle to target", getAngleToTarget(Field.getAllianceHubPose()), true);
        partsNT.putBoolean("Turret Debug Active", debug, !RobotConstants.COMPETITION);
    }

    @Override
    public void stop() {
        turretState = TurretState.DISABLED;
    }

    @Override
    public void reset() {
        turretState = TurretState.IDLE;
    }

    @Override
    public void log() {
        partsLogger.logString("Turret State", turretState.toString(), RobotContainer.debug || debug);
    }

    @Override
    public void periodic() {
        if (RobotContainer.debug || debug) {
            setSpeed(partsNT.getDouble("Turret Speed", true));
            turretPIDController.setGoal(partsNT.getDouble("Turret Angle", true));
            double pidCalc = turretPIDController.calculate(getAngle(), partsNT.getDouble("Turret Angle", true));
            // double ffCalc =
            // turretFeedforward.calculate(turretPIDController.getSetpoint());

            double voltage = pidCalc; // + ffCalc;

            setVoltage(voltage);
        } else {
            double voltage = 0;

            switch (turretState) {
                case DISABLED:
                case IDLE:
                    setSpeed(0);
                    break;
                case TRACKING:
                case TRACKING_CORNER:
                    Pose2d target = getTargetPose();
                    if (isValidAngle()) {
                        turretPIDController.setGoal(getAngleToTarget(target));
                        double pidCalc = turretPIDController.calculate(getAngle(), getAngleToTarget(target));
                        // double ffCalc =
                        // turretFeedforward.calculate(turretPIDController.getSetpoint());

                        partsNT.putDouble("Turret voltage", voltage, RobotContainer.debug || debug);
                        partsNT.putBoolean("Turret at setpoint", turretPIDController.atSetpoint(), RobotContainer.debug || debug);

                        voltage = pidCalc; // + ffCalc;

                        setVoltage(voltage);
                    } else {
                        setSpeed(0);
                    }
                    break;
                
                    case LEFT_CORNER:
                    case RIGHT_CORNER:
                        turretPIDController.setGoal(turretState.getAngle());
                        double pidCalc = turretPIDController.calculate(getAngle(), turretState.getAngle());
                        // double ffCalc =
                        // turretFeedforward.calculate(turretPIDController.getSetpoint());

                        partsNT.putDouble("Turret voltage", voltage, RobotContainer.debug || debug);
                        partsNT.putBoolean("Turret at setpoint", turretPIDController.atSetpoint(), RobotContainer.debug || debug);

                        voltage = pidCalc; // + ffCalc;

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
     * Sets the speed of the Turret.
     * 
     * @param speed The speed between <code>-1.0</code> and <code>1.0</code>.
     */
    protected abstract void setSpeed(double speed);

    protected abstract void setVoltage(double voltage);

    protected abstract double getVoltage();

    protected abstract double getAngle();

    public boolean isValidAngle() {
        return Math.abs(getAngleToTarget(getTargetPose())) <= 100;
    }

    public boolean atSetpoint() {
        return turretPIDController.atSetpoint();
    }

    public TurretState getState() {
        return turretState;
    }

    public Command track() {
        return PARTsCommandUtils.setCommandName("Turret.track", this.runOnce(() -> {
            turretState = TurretState.TRACKING;
        }));
    }

    public Command trackCorner() {
        return PARTsCommandUtils.setCommandName("Turret.track", this.runOnce(() -> {
            turretState = TurretState.TRACKING_CORNER;
        }));
    }

    public Command rightCorner() {
        return PARTsCommandUtils.setCommandName("Turret.track", this.runOnce(() -> {
            turretState = TurretState.RIGHT_CORNER;
        }));
    }

    public Command leftCorner() {
        return PARTsCommandUtils.setCommandName("Turret.track", this.runOnce(() -> {
            turretState = TurretState.LEFT_CORNER;
        }));
    }

    public Command idle() {
        return PARTsCommandUtils.setCommandName("Turret.idle", this.runOnce(() -> {
            turretState = TurretState.IDLE;
        }));
    }

    public boolean withinSetpointRange() {
        return Math.abs(turretPIDController.getSetpoint().position - getAngle()) < 5;
    }

    public Pose2d getTargetPose() {
        return turretState == TurretState.TRACKING ? Field.getAllianceHubPose() : Field.getNearestAllianceCorner(robotPoseSupplier.get());

    }
    // endregion

    // region private functions
    private double getAngleToTarget(Pose2d target) {
        double angleToTarget = edu.wpi.first.math.MathUtil
                .inputModulus(robotPoseSupplier.get().getRotation().getDegrees(), -180, 180)
                - (Math.atan2(target.getY() - robotPoseSupplier.get().getY(),
                        target.getX() - robotPoseSupplier.get().getX()) * 180 / Math.PI);
        return angleToTarget;
    }
    // endregion private functions
}
