package frc.robot.subsystems.Turret;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.TurretConstants;
import frc.robot.constants.TurretConstants.TurretState;
import frc.robot.util.Field;

import java.util.function.Supplier;

import org.parts3492.partslib.command.PARTsCommandUtils;
import org.parts3492.partslib.command.PARTsSubsystem;

public abstract class Turret extends PARTsSubsystem {
    private TurretState turretState = TurretState.IDLE;

    private ProfiledPIDController turretPIDController;
    private SimpleMotorFeedforward turretFeedforward;
    private Supplier<Pose2d> robotPoseSupplier;

    public Turret(Supplier<Pose2d> robotPoseSupplier) {
        super("Turret", RobotConstants.LOGGING);
        if (RobotConstants.DEBUGGING) {
            partsNT.putDouble("Turret Speed", 0);
        }

        this.robotPoseSupplier = robotPoseSupplier;

        turretPIDController = new ProfiledPIDController(TurretConstants.P, TurretConstants.I, TurretConstants.D, new TrapezoidProfile.Constraints(TurretConstants.TURRET_MAX_VELOCITY, TurretConstants.TURRET_MAX_ACCELERATION));
        turretFeedforward = new SimpleMotorFeedforward(TurretConstants.S, TurretConstants.V, TurretConstants.A);

        turretPIDController.setTolerance(TurretConstants.PID_THRESHOLD);
    }

    // region Generic Subsystem Functions
    @Override
    public void outputTelemetry() {
        partsNT.putString("Turret State", turretState.toString());
        partsNT.putDouble("Angle", getAngle());
        partsNT.putDouble("Voltage", getVoltage());
        partsNT.putDouble("Get Setpoint", turretPIDController.getSetpoint().position);
        partsNT.putBoolean("At Setpoint", turretPIDController.atSetpoint());
        partsNT.putDouble("Current Error", turretPIDController.getPositionError());
        partsNT.putDouble("Get Angle to Turret", getAngleToTarget());
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
        partsLogger.logString("Turret State", turretState.toString());
    }

    @Override
    public void periodic() {
        if (RobotConstants.DEBUGGING) {
            setSpeed(partsNT.getDouble("Turret Speed"));
        } else {
            double voltage = 0;

            switch (turretState) {
                case DISABLED:
                case IDLE:
                    setSpeed(0);
                    break;
                case TRACKING:
                    if (isValidAngle()) {
                        turretPIDController.setGoal(getAngleToTarget());
                        double pidCalc = turretPIDController.calculate(getAngle(), getAngleToTarget());
                        // double ffCalc =
                        // turretFeedforward.calculate(turretPIDController.getSetpoint());

                        partsNT.putDouble("Turret voltage", voltage);
                        partsNT.putBoolean("Turret at setpoint", turretPIDController.atSetpoint());

                        voltage = pidCalc; // + ffCalc;

                        setVoltage(voltage);
                    } else {
                        setSpeed(0);
                    }
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
        return Math.abs(getAngleToTarget()) <= 100;
    }

    public TurretState getState() {
        return turretState;
    }

    public Command track() {
        return PARTsCommandUtils.setCommandName("Turret.track", this.runOnce(() -> {
            turretState = TurretState.TRACKING;
        }));
    }

    public Command idle() {
        return PARTsCommandUtils.setCommandName("Turret.idle", this.runOnce(() -> {
            turretState = TurretState.IDLE;
        }));
    }
    // endregion

    // region private functions
    private double getAngleToTarget() {
        double angleToTarget = edu.wpi.first.math.MathUtil
                .inputModulus(robotPoseSupplier.get().getRotation().getDegrees(), -180, 180)
                - (Math.atan2(Field.getAllianceHubPose().getY() - robotPoseSupplier.get().getY(),
                        Field.getAllianceHubPose().getX() - robotPoseSupplier.get().getX()) * 180 / Math.PI);
        return angleToTarget;
    }
    // endregion private functions
}
