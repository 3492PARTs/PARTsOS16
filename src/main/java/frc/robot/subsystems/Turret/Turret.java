package frc.robot.subsystems.Turret;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.TurretConstants;
import frc.robot.states.TurretState;
import frc.robot.util.Field;
import frc.robot.util.Hub;

import java.util.function.Supplier;

import org.parts3492.partslib.PARTsUnit.PARTsUnitType;
import org.parts3492.partslib.command.PARTsCommandUtils;
import org.parts3492.partslib.command.PARTsSubsystem;

public abstract class Turret extends PARTsSubsystem {
    private TurretState turretState = TurretState.IDLE;

    private PIDController turretPIDController;
    private SimpleMotorFeedforward turretFeedforward;
    private Supplier<Pose2d> robotPoseSupplier;

    public Turret(Supplier<Pose2d> robotPoseSupplier) {
        super("Turret", RobotConstants.LOGGING);
        /*
         * if (RobotConstants.DEBUGGING) {
         * partsNT.putDouble("Turret Speed", 0);
         * }
         */

        this.robotPoseSupplier = robotPoseSupplier;

        turretPIDController = new PIDController(TurretConstants.P, TurretConstants.I, TurretConstants.D);
        turretFeedforward = new SimpleMotorFeedforward(TurretConstants.S, TurretConstants.V, TurretConstants.A);

        turretPIDController.setTolerance(TurretConstants.PID_THRESHOLD);
    }

    // region Generic Subsystem Functions
    @Override
    public void outputTelemetry() {
        partsNT.putString("Turret State", turretState.toString());
        partsNT.putDouble("Angle", getAngle());
        partsNT.putDouble("Voltage", getVoltage());
        partsNT.putDouble("Get Setpoint", turretPIDController.getSetpoint());
        partsNT.putBoolean("At Setpoint", turretPIDController.atSetpoint());
        partsNT.putDouble("Current Error", turretPIDController.getError());
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
    // endregion

    @Override
    public void periodic() {
        if (RobotConstants.DEBUGGING) {
            setSpeed(partsNT.getDouble("Turret Speed"));
        } else {
            double voltage = 0;
            turretPIDController.setSetpoint(turretState.getAngle());

            double pidCalc = turretPIDController.calculate(getAngle(), turretState.getAngle());
            double ffCalc = turretFeedforward.calculate(turretPIDController.getSetpoint());

            voltage = pidCalc + ffCalc;

            setVoltage(voltage);
        }
    }

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

    public TurretState getState() {
        return turretState;
    }

    public Command track() {
        return PARTsCommandUtils.setCommandName("Command Track", this.runOnce(() -> {
            turretState = TurretState.TRACKING;
        }));
    }

    public Command idle() {
        return PARTsCommandUtils.setCommandName("Command Idle", this.runOnce(() -> {
            turretState = TurretState.IDLE;
        }));
    }
    // endregion

    // region private functions
    private double getAngleToTarget() {
        /*
         * Pose2d targetPose = Field.getAllianceHubPose();
         * Pose2d robotPose = robotPoseSupplier.get();
         * Rotation2d angleToTarget =
         * targetPose.getTranslation().minus(robotPose.getTranslation()).getAngle();
         */
        double angleToTarget = edu.wpi.first.math.MathUtil
                .inputModulus(robotPoseSupplier.get().getRotation().getDegrees(), -180, 180)
                - (Math.atan2(Field.getAllianceHubPose().getY() - robotPoseSupplier.get().getY(),
                        Field.getAllianceHubPose().getX() - robotPoseSupplier.get().getX()) * 180 / Math.PI);
        return angleToTarget;

        /*if (angleToTarget > 180) {
            return 180 - angleToTarget;
        } else if (angleToTarget < -180) {
            return 360 + angleToTarget;
        } else {
            return angleToTarget;
        }*/
    }
    // endregion private functions
}
