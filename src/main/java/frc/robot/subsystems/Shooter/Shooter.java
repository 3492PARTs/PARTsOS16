package frc.robot.subsystems.Shooter;

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
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.ShooterConstants.ShooterState;
import frc.robot.constants.TurretConstants;
import frc.robot.constants.TurretConstants.TurretState;
import frc.robot.subsystems.drive.PARTsDrivetrain.RobotVelocitySupplier;
import frc.robot.util.Field;
import frc.robot.util.Hub.Targets;
import frc.robot.util.SOTMCalculator;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.parts3492.partslib.PARTsUnit.PARTsUnitType;
import org.parts3492.partslib.command.PARTsCommandUtils;
import org.parts3492.partslib.command.PARTsSubsystem;

public class Shooter extends PARTsSubsystem {
  private ShooterState shooterState = ShooterState.IDLE;

  private PIDController shooterPIDController;
  private SimpleMotorFeedforward shooterFeedforward;
  private Supplier<Pose2d> robotPoseSupplier;
  private Supplier<TurretState> turretStateSupplier;
  private RobotVelocitySupplier velocitySupplier;
  private FieldObject2d calculatedRobotPose;
  private ShooterIO io;
  private ShooterInputsAutoLogged shooterInputs = new ShooterInputsAutoLogged();

  protected boolean debug = false;
  private Command toggleDebug =
      Commands.runOnce(
              () -> {
                debug = !debug;
                partsNT.putDouble("Shooter Speed", 0, true);
              })
          .ignoringDisable(true);

  private double offset = 0;

  /**
   * Creates a new Shooter subsystem.
   *
   * @param poseSupplier The supplier for the robot's pose, used to get the distance to the hub and
   *     trench.
   */
  public Shooter(
      ShooterIO io,
      Supplier<Pose2d> poseSupplier,
      RobotVelocitySupplier velocitySupplier,
      Supplier<TurretState> turretSupplierState) {
    super("Shooter", RobotConstants.LOGGING);
    if (RobotConstants.COMPETITION) debug = false;

    this.robotPoseSupplier = poseSupplier;
    this.turretStateSupplier = turretSupplierState;
    this.velocitySupplier = velocitySupplier;
    this.io = io;

    calculatedRobotPose = Field.FIELD2D.getObject("Calculated Robot Pose");

    if (RobotContainer.debug || debug) {
      partsNT.putDouble("Shooter Speed", 0, true);
    }

    shooterPIDController =
        new PIDController(ShooterConstants.P, ShooterConstants.I, ShooterConstants.D);
    shooterFeedforward =
        new SimpleMotorFeedforward(ShooterConstants.S, ShooterConstants.V, ShooterConstants.A);
    shooterPIDController.setTolerance(ShooterConstants.PID_THRESHOLD);

    partsNT.putSmartDashboardSendable(
        "Toggle Shooter Debug", toggleDebug, !RobotConstants.COMPETITION);
    putOffsetOnNT();
  }

  // region Generic Subsystem Functions
  @Override
  public void outputTelemetry() {
    partsNT.putString("Shooter State", shooterState.toString(), !RobotConstants.COMPETITION);
    partsNT.putDouble("RPM", getRPM(), true);
    partsNT.putDouble("Voltage", getVoltage(), RobotContainer.debug || debug);
    partsNT.putDouble(
        "Get Setpoint", shooterPIDController.getSetpoint(), RobotContainer.debug || debug);
    partsNT.putBoolean("At Setpoint", shooterPIDController.atSetpoint(), true);
    partsNT.putDouble(
        "Current Error", shooterPIDController.getError(), RobotContainer.debug || debug);
    partsNT.putBoolean("Shooter Debug Active", debug, !RobotConstants.COMPETITION);
    partsNT.putDouble("Offset", offset, true);
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
  public void log() {}

  @Override
  public void periodic() {
    io.updateInputs(shooterInputs);
    org.littletonrobotics.junction.Logger.processInputs("Shooter", shooterInputs);
    outputTelemetry();
    if (RobotContainer.debug || debug) {
      double rpm = partsNT.getDouble("Shooter Speed", true);
      setVoltage(calculateRPMVoltage(rpm));
    } else {
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

      double shooterRPM =
          (shooterState == ShooterState.MANUAL)
              ? shooterState.getRPM()
              : SOTMCalculator.getRPMToGoal(calcTurretPose, Field.getAllianceHubPose());

      /*
       * if (!RobotConstants.COMPETITION) {
       * calculatedRobotPose.setPose(calcRobotPose);
       * }
       */
      /*
       * boolean inTrench = Trench.isUnderTrench(robotPoseSupplier.get());
       *
       * if (inTrench && Math.abs(drivetrain.getXVelocity().getValue()) < 1.5
       * && Math.abs(drivetrain.getYVelocity().getValue()) < 1.5) {
       * shooterRPM = ShooterState.getZoneRPM(Targets.TRENCH);
       * }
       */

      if (shooterRPM == 0 && turretStateSupplier.get() == TurretState.TRACKING_CORNER) {
        shooterRPM = ShooterState.getZoneRPM(Targets.BEHIND_HUB);
      }

      if (turretStateSupplier.get() == TurretState.TRACKING_CORNER) {
        shooterRPM += 300;
      }

      shooterRPM += offset;

      partsNT.putDouble("Shooting RPM", shooterRPM, true);

      switch (shooterState) {
        case DISABLED:
        case IDLE:
          setSpeed(0);
          break;
        case SHOOTING:
        case MANUAL:
          if (debug) {
            shooterRPM = partsNT.getDouble("Shooter Speed", true);
          }
          setVoltage(calculateRPMVoltage(shooterRPM));
          break;
        case TEST:
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
  public void setSpeed(double speed) {
    io.setSpeed(speed);
  }

  /**
   * Sets the voltage of the Shooter.
   *
   * @param voltage The voltage between <code>-12.0</code> and <code>12.0</code>.
   */
  public void setVoltage(double voltage) {
    io.setVoltage(voltage);
  }

  /**
   * Gets the voltage of the Shooter.
   *
   * @return The voltage between <code>-12.0</code> and <code>12.0</code>.
   */
  public double getVoltage() {
    return io.getVoltage();
  }

  /**
   * Gets the RPM of the Shooter.
   *
   * @return The RPM of the Shooter.
   */
  public double getRPM() {
    return io.getRPM();
  }

  /**
   * Gets the current state of the Shooter.
   *
   * @return The current state of the Shooter.
   */
  public ShooterState getState() {
    return shooterState;
  }

  public void setState() {
    shooterState = ShooterState.TEST;
  }

  /**
   * Command to set the Shooter to the {@link ShooterState#SHOOTING SHOOTING} state.
   *
   * @return The command.
   */
  public Command shoot() {
    return PARTsCommandUtils.setCommandName(
        "Shooter.shoot",
        this.runOnce(
            () -> {
              shooterState = ShooterState.SHOOTING;
            }));
  }

  /**
   * Command to set the Shooter to the {@link ShooterState#IDLE IDLE} state.
   *
   * @return The command.
   */
  public Command idle() {
    return PARTsCommandUtils.setCommandName(
        "Shooter.idle",
        this.runOnce(
            () -> {
              shooterState = ShooterState.IDLE;
            }));
  }

  /**
   * Command to set the Shooter to the {@link ShooterState#MANUAL MANUAL} state.
   *
   * <p>This allows manual control of the shooter RPM.
   *
   * @return The command.
   */
  public Command manualShoot() {
    return PARTsCommandUtils.setCommandName(
        "Shooter.manualShoot",
        this.runOnce(
            () -> {
              shooterState = ShooterState.MANUAL;
            }));
  }

  /**
   * Gets whether the Shooter is at the setpoint RPM.
   *
   * @return A boolean supplier that returns true if the Shooter is at the setpoint RPM.
   */
  public BooleanSupplier atSetpoint() {
    return () -> shooterPIDController.atSetpoint();
  }

  /**
   * Gets the current setpoint of the Shooter.
   *
   * @return A double supplier that returns the current setpoint of the Shooter in RPM.
   */
  public DoubleSupplier getSetpoint() {
    return () -> shooterPIDController.getSetpoint();
  }

  /**
   * Gets whether the Shooter is within a certain range of the setpoint RPM.
   *
   * @return Boolean that returns true if the Shooter is within the setpoint range.
   */
  public boolean withinSetpointRange() {
    return Math.abs(shooterPIDController.getSetpoint() - getRPM()) < 300;
  }

  public Command setSpeedOffset(DoubleSupplier d) {
    return PARTsCommandUtils.setCommandName(
        "Shooter.addSpeed",
        Commands.runOnce(
            () -> {
              this.offset = d.getAsDouble();
              putOffsetOnNT();
            }));
  }

  public double getSpeedOffset() {
    return offset;
  }

  // endregion

  private double calculateRPMVoltage(double rpm) {
    shooterPIDController.setSetpoint(rpm);
    double pidCalc = shooterPIDController.calculate(getRPM(), rpm);
    double ffCalc =
        shooterFeedforward.calculate(
            (shooterPIDController.getSetpoint()
                    * Math.PI
                    * ShooterConstants.SHOOTER_WHEEL_RADIUS.to(PARTsUnitType.Meter)
                    * 2)
                / 60);

    double voltage = pidCalc + ffCalc;

    return voltage;
  }

  private void putOffsetOnNT() {
    partsNT.putDouble("Offset", offset, true);
  }
}
