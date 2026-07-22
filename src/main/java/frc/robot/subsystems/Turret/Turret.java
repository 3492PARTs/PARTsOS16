package frc.robot.subsystems.Turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.TurretConstants;
import frc.robot.constants.TurretConstants.TurretState;
import frc.robot.subsystems.drive.PARTsDrivetrain.RobotVelocitySupplier;
import frc.robot.util.Field;
import frc.robot.util.SOTMCalculator;
import java.util.function.Supplier;
import org.parts3492.partslib.PARTsUnit.PARTsUnitType;
import org.parts3492.partslib.command.PARTsCommandUtils;
import org.parts3492.partslib.command.PARTsSubsystem;

public class Turret extends PARTsSubsystem {
  private TurretState turretState = TurretState.IDLE;

  private PIDController turretPIDController;
  private SimpleMotorFeedforward turretFeedforward;
  private Supplier<Pose2d> robotPoseSupplier;
  private RobotVelocitySupplier velocitySupplier;
  private FieldObject2d fieldTarget;
  private FieldObject2d projectedRobotPose;
  private TurretIO io;

  protected boolean debug = false;
  private Command toggleDebug =
      Commands.runOnce(
              () -> {
                debug = !debug;
                partsNT.putDouble("Turret Speed", 0, !RobotConstants.COMPETITION);
                partsNT.putDouble("Turret Angle", 0, !RobotConstants.COMPETITION);
              })
          .ignoringDisable(true);

  public Turret(
      TurretIO io, Supplier<Pose2d> robotPoseSupplier, RobotVelocitySupplier velocitySupplier) {
    super("Turret", RobotConstants.LOGGING);
    if (RobotConstants.COMPETITION) debug = false;

    if (RobotContainer.debug || debug) {
      partsNT.putDouble("Turret Speed", 0, !RobotConstants.COMPETITION);
      partsNT.putDouble("Turret Angle", 0, !RobotConstants.COMPETITION);
    }

    this.robotPoseSupplier = robotPoseSupplier;
    this.velocitySupplier = velocitySupplier;
    this.io = io;
    fieldTarget = Field.FIELD2D.getObject("Turret Target");
    projectedRobotPose = Field.FIELD2D.getObject("Projected Robot Pose");

    turretPIDController =
        new PIDController(TurretConstants.P, TurretConstants.I, TurretConstants.D);
    turretFeedforward =
        new SimpleMotorFeedforward(TurretConstants.S, TurretConstants.V, TurretConstants.A);
    turretPIDController.setTolerance(TurretConstants.PID_THRESHOLD);

    partsNT.putSmartDashboardSendable(
        "Toggle Turret Debug", toggleDebug, !RobotConstants.COMPETITION);
  }

  // region Generic Subsystem Functions
  @Override
  public void outputTelemetry() {
    partsNT.putBoolean("Valid Angle", isValidAngle(), true);
    partsNT.putString("Turret State", turretState.toString(), !RobotConstants.COMPETITION);
    partsNT.putDouble("Angle", getAngle(), true);
    partsNT.putDouble("Voltage", getVoltage(), RobotContainer.debug || debug);
    partsNT.putDouble(
        "Get Setpoint", turretPIDController.getSetpoint(), RobotContainer.debug || debug);
    partsNT.putBoolean(
        "At Setpoint", turretPIDController.atSetpoint(), !RobotConstants.COMPETITION);
    partsNT.putDouble(
        "Current Error", turretPIDController.getPositionError(), RobotContainer.debug || debug);
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
  public void log() {}

  @Override
  public void periodic() {
    if (RobotContainer.debug || debug) {
      setVoltage(calculateVoltage(partsNT.getDouble("Turret Angle", true)));
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
            partsNT.putDouble("Turret voltage", voltage, RobotContainer.debug || debug);
            partsNT.putBoolean(
                "Turret at setpoint",
                turretPIDController.atSetpoint(),
                RobotContainer.debug || debug);

            setVoltage(calculateVoltage(getAngleToTarget(target)));
          } else {
            setSpeed(0);
          }
          break;

        case LEFT_CORNER:
        case RIGHT_CORNER:
          partsNT.putDouble("Turret voltage", voltage, RobotContainer.debug || debug);
          partsNT.putBoolean(
              "Turret at setpoint",
              turretPIDController.atSetpoint(),
              RobotContainer.debug || debug);

          setVoltage(calculateVoltage(turretState.getAngle()));
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
  protected void setSpeed(double speed) {
    io.setSpeed(speed);
  }

  protected void setVoltage(double voltage) {
    io.setVoltage(voltage);
  }

  protected double getVoltage() {
    return io.getVoltage();
  }

  protected double getAngle() {
    return io.getAngle();
  }

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
    return PARTsCommandUtils.setCommandName(
        "Turret.track",
        this.runOnce(
            () -> {
              turretState = TurretState.TRACKING_HUB;
            }));
  }

  public Command trackCorner() {
    return PARTsCommandUtils.setCommandName(
        "Turret.track",
        this.runOnce(
            () -> {
              turretState = TurretState.TRACKING_CORNER;
            }));
  }

  public Command rightCorner() {
    return PARTsCommandUtils.setCommandName(
        "Turret.track",
        this.runOnce(
            () -> {
              turretState = TurretState.RIGHT_CORNER;
            }));
  }

  public Command leftCorner() {
    return PARTsCommandUtils.setCommandName(
        "Turret.track",
        this.runOnce(
            () -> {
              turretState = TurretState.LEFT_CORNER;
            }));
  }

  public Command idle() {
    return PARTsCommandUtils.setCommandName(
        "Turret.idle",
        this.runOnce(
            () -> {
              turretState = TurretState.IDLE;
            }));
  }

  public boolean withinSetpointRange() {
    return Math.abs(turretPIDController.getSetpoint() - getAngle()) < 10;
  }

  public Pose2d getTargetPose() {
    Pose2d target =
        turretState == TurretState.TRACKING_HUB
            ? Field.getAllianceHubPose()
            : Field.getNearestAllianceCorner(robotPoseSupplier.get());
    fieldTarget.setPose(target);
    return target;
  }

  private double calculateVoltage(double angle) {
    turretPIDController.setSetpoint(angle);
    double pidCalc = turretPIDController.calculate(getAngle(), angle);
    // double ffCalc = (turretPIDController.atSetpoint()) ? 0 :
    // turretFeedforward.calculate(turretPIDController.getSetpoint() * Math.PI / 180);
    double voltage = pidCalc;
    return MathUtil.clamp(voltage, -8, 8);
  }

  // endregion

  // region private functions
  protected double getAngleToTarget(Pose2d target) {
    Transform2d robotVelocity = velocitySupplier.get();
    Pose2d calcTurretPose =
        SOTMCalculator.collapsePose(
            robotPoseSupplier
                .get()
                .plus(
                    new Transform2d(
                        TurretConstants.TURRET_OFFSET_CENTER.to(PARTsUnitType.Meter),
                        0,
                        new Rotation2d())),
            robotVelocity);

    if (!RobotConstants.COMPETITION) {
      projectedRobotPose.setPose(calcTurretPose);
    }
    double angleToTarget =
        calcTurretPose.getRotation().getDegrees()
            - (Math.atan2(
                    target.getY() - calcTurretPose.getY(), target.getX() - calcTurretPose.getX())
                * 180
                / Math.PI);
    if (angleToTarget <= -180) {
      angleToTarget += 360;
    } else if (angleToTarget >= 180) {
      angleToTarget -= 360;
    }
    return angleToTarget;
  }

  // endregion private functions
}
