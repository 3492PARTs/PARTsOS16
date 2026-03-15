package frc.robot.subsystems.Turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.TurretConstants;
import frc.robot.constants.ShooterConstants.ShooterState;
import frc.robot.constants.TurretConstants.TurretState;
import frc.robot.subsystems.Drivetrain.PARTsDrivetrain;
import frc.robot.util.Field;
import frc.robot.util.Hub;
import frc.robot.util.Hub.Targets;

import java.util.function.Supplier;

import org.parts3492.partslib.command.PARTsCommandUtils;
import org.parts3492.partslib.command.PARTsSubsystem;

public abstract class Turret extends PARTsSubsystem {
    private TurretState turretState = TurretState.IDLE;

    private PIDController turretPIDController;
    private Supplier<Pose2d> robotPoseSupplier;
    private PARTsDrivetrain drivetrain;
    private FieldObject2d fieldTarget;

    protected boolean debug = false;
    private Command toggleDebug = Commands.runOnce(()-> debug = !debug).ignoringDisable(true);

    public Turret(Supplier<Pose2d> robotPoseSupplier, PARTsDrivetrain drivetrain) {
        super("Turret", RobotConstants.LOGGING);
        if (RobotConstants.COMPETITION) debug = false;

        if (RobotContainer.debug || debug) {
         partsNT.putDouble("Turret Speed", 0, !RobotConstants.COMPETITION);
         partsNT.putDouble("Turret Angle", 0, !RobotConstants.COMPETITION);
        }

        this.robotPoseSupplier = robotPoseSupplier;
        this.drivetrain = drivetrain;
        fieldTarget = Field.FIELD2D.getObject("Turret Target");

        turretPIDController = new PIDController(TurretConstants.P, TurretConstants.I, TurretConstants.D);
        turretPIDController.setTolerance(TurretConstants.PID_THRESHOLD);

        partsNT.putSmartDashboardSendable("Toggle Turret Debug", toggleDebug, !RobotConstants.COMPETITION);
    }

    // region Generic Subsystem Functions
    @Override
    public void outputTelemetry() {
        partsNT.putBoolean("Valid Angle", isValidAngle(), true);
        partsNT.putString("Turret State", turretState.toString(), !RobotConstants.COMPETITION);
        partsNT.putDouble("Angle", getAngle(), true);
        partsNT.putDouble("Voltage", getVoltage(), RobotContainer.debug || debug);
        partsNT.putDouble("Get Setpoint", turretPIDController.getSetpoint(), RobotContainer.debug || debug);
        partsNT.putBoolean("At Setpoint", turretPIDController.atSetpoint(), !RobotConstants.COMPETITION);
        partsNT.putDouble("Current Error", turretPIDController.getPositionError(), RobotContainer.debug || debug);
        partsNT.putDouble("Get Angle to target", getAngleToTarget(getTargetPose()), true);
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
            turretPIDController.setSetpoint(partsNT.getDouble("Turret Angle", true));
            double pidCalc = turretPIDController.calculate(getAngle(), partsNT.getDouble("Turret Angle", true));

            double voltage = MathUtil.clamp(pidCalc, -9, 9); 
            setVoltage(voltage);
        } else {
            double voltage = 0;

            switch (turretState) {
                case DISABLED:
                case IDLE:
                    setSpeed(0);
                    break;
                case TRACKING_HUB:
                case TRACKING_CORNER:
                    Pose2d target = getTargetPose();
                    if (isValidAngle()) {
                        turretPIDController.setSetpoint(getAngleToTarget(target));
                        double pidCalc = turretPIDController.calculate(getAngle(), getAngleToTarget(target));

                        partsNT.putDouble("Turret voltage", voltage, RobotContainer.debug || debug);
                        partsNT.putBoolean("Turret at setpoint", turretPIDController.atSetpoint(), RobotContainer.debug || debug);

                        voltage = MathUtil.clamp(pidCalc, -9, 9);

                        setVoltage(voltage);
                    } else {
                        setSpeed(0);
                    }
                    break;
                
                    case LEFT_CORNER:
                    case RIGHT_CORNER:
                        turretPIDController.setSetpoint(turretState.getAngle());
                        double pidCalc = turretPIDController.calculate(getAngle(), turretState.getAngle());

                        partsNT.putDouble("Turret voltage", voltage, RobotContainer.debug || debug);
                        partsNT.putBoolean("Turret at setpoint", turretPIDController.atSetpoint(), RobotContainer.debug || debug);

                        voltage = MathUtil.clamp(pidCalc, -9, 9);

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

    public Command trackHub() {
        return PARTsCommandUtils.setCommandName("Turret.track", this.runOnce(() -> {
            turretState = TurretState.TRACKING_HUB;
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
        return Math.abs(turretPIDController.getSetpoint() - getAngle()) < 5;
    }

    public Pose2d getTargetPose() {
        return turretState == TurretState.TRACKING_HUB ? Field.getAllianceHubPose() : Field.getNearestAllianceCorner(robotPoseSupplier.get());

    }

    // endregion

    // region private functions
    private double getAngleToTarget(Pose2d target) {
        Targets zone = Hub.getZone(robotPoseSupplier.get());
        double timeOfFlight = (zone == null) ? 0 : ShooterState.getTofFromDistanceToHub(robotPoseSupplier.get());
        Pose2d calculatedPose = 
                    target.plus(new Transform2d(drivetrain.getXVelocity().getValue() * timeOfFlight,
                            drivetrain.getYVelocity().getValue() * timeOfFlight, new Rotation2d()));
        if (!RobotConstants.COMPETITION) {
            fieldTarget.setPose(calculatedPose);
        }
        double angleToTarget = robotPoseSupplier.get().getRotation().getDegrees()
                - (Math.atan2(calculatedPose.getY() - robotPoseSupplier.get().getY(),
                        calculatedPose.getX() - robotPoseSupplier.get().getX()) * 180 / Math.PI);
        if (angleToTarget <= -180) {
            angleToTarget += 360;
        }
        else if (angleToTarget >= 180) {
            angleToTarget -= 360;
        }
        return angleToTarget;
    }
    // endregion private functions
}
