package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.constants.RobotConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.util.Field;
import java.io.IOException;
import java.util.Arrays;
import java.util.HashSet;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.json.simple.parser.ParseException;
import org.parts3492.partslib.PARTsUnit;
import org.parts3492.partslib.PARTsUnit.PARTsUnitType;
import org.parts3492.partslib.RobotUtils;
import org.parts3492.partslib.command.IPARTsSubsystem;
import org.parts3492.partslib.command.PARTsCommandUtils;

public class PARTsDrivetrain extends Drive implements IPARTsSubsystem {
  /*-------------------------------- Private instance variables ---------------------------------*/
  private boolean fineGrainDrive = false;
  private double MaxSpeed =
      TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top
  // speed
  private double MaxAngularRate =
      RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per
  // second max angular velocity

  // show robot pose on field dashboard
  private FieldObject2d robotFieldObject2d;

  // pid controllers
  private ProfiledPIDController thetaController;
  private ProfiledPIDController xRangeController;
  private ProfiledPIDController yRangeController;

  private boolean isControlledRotationEnabled = false;

  // --- Airborne / pose-freeze gating ---
  private boolean isAirborne = false;
  private int airborneDebounceCycles = 0;
  private int stableDebounceCycles = 0;
  private GyroIO gyroIO;

  public PARTsDrivetrain(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {
    super(gyroIO, flModuleIO, frModuleIO, blModuleIO, brModuleIO);
    this.gyroIO = gyroIO;
    initializeControllers();
    robotFieldObject2d = Field.FIELD2D.getRobotObject();
  }

  // region Generic Subsystem Functions
  @Override
  public void outputTelemetry() {}

  @Override
  public void reset() {}

  @Override
  public void log() {}

  @Override
  public void periodic() {
    updateAirborneState();

    super.periodic();

    robotFieldObject2d.setPose(getPose());
  }

  // endregion

  // region Custom Public Functions

  public Transform2d getRobotVelocity() {
    return new Transform2d(
        super.getChassisSpeeds().vxMetersPerSecond,
        super.getChassisSpeeds().vyMetersPerSecond,
        new Rotation2d());
  }

  public SwerveRequest.FieldCentric getFieldCentricDriveRequest() {
    /* Setting up bindings for necessary control of the swerve drive platform */
    return new SwerveRequest.FieldCentric()
        .withDeadband(MaxSpeed * 0.1)
        .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10%
        // deadband
        .withDesaturateWheelSpeeds(true)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for
    // drive
  }

  public SwerveRequest.SwerveDriveBrake getBrakeDriveRequest() {
    return new SwerveRequest.SwerveDriveBrake();
  }

  public SwerveRequest.PointWheelsAt getPointDriveRequest() {
    return new SwerveRequest.PointWheelsAt();
  }

  public void toggleFineGrainDrive() {
    fineGrainDrive = !fineGrainDrive;
    outputTelemetry();
  }

  public Pose2d getFieldCentricPose() {
    return Field.conditionallyTransformToOppositeAlliance(getPose());
  }

  public Command commandPathFindToPath(String pathname) {
    try {
      // Load the path we want to pathfind to and follow
      PathPlannerPath path = PathPlannerPath.fromPathFile(pathname);

      // Create the constraints to use while pathfinding. The constraints defined in
      // the path will only be used for the path.
      PathConstraints constraints =
          new PathConstraints(
              2,
              2,
              PARTsUnit.DegreesToRadians.apply(540.0),
              PARTsUnit.DegreesToRadians.apply(720.0));

      // Since AutoBuilder is configured, we can use it to build pathfinding commands
      Command pathfindingCommand = AutoBuilder.pathfindThenFollowPath(path, constraints);
      return PARTsCommandUtils.setCommandName(
          "PARTsDrivetrain.commandPathFindToPath", pathfindingCommand);

    } catch (IOException e) {
      e.printStackTrace();
    } catch (FileVersionException e) {
      // Auto-generated catch block
      e.printStackTrace();
    } catch (ParseException e) {
      // Auto-generated catch block
      e.printStackTrace();
    }
    return PARTsCommandUtils.setCommandName(
        "PARTsDrivetrain.commandPathFindToPath.wait", new WaitCommand(0));
  }

  public Command commandPathFindToPose(Pose2d pose) {

    // Create the constraints to use while pathfinding. The constraints defined in
    // the path will only be used for the path.
    PathConstraints constraints =
        new PathConstraints(
            DrivetrainConstants.MAX_AIM_VELOCITY,
            DrivetrainConstants.MAX_AIM_ACCELERATION,
            PARTsUnit.DegreesToRadians.apply(540.0),
            PARTsUnit.DegreesToRadians.apply(720.0));

    // Since AutoBuilder is configured, we can use it to build pathfinding commands
    Command pathfindingCommand =
        AutoBuilder.pathfindToPose(
            Field.conditionallyTransformToOppositeAlliance(pose),
            constraints,
            0.0); // Goal end velocity in meters/sec
    return PARTsCommandUtils.setCommandName(
        "PARTsDrivetrain.commandPathFindToPose", pathfindingCommand);
  }

  public Command commandPathOnTheFly(Pose2d pose) {

    return PARTsCommandUtils.setCommandName(
        "PARTsDrivetrain.commandPathOnTheFly",
        Commands.defer(
            () -> {
              PathConstraints constraints =
                  new PathConstraints(2, 2, 2 * Math.PI, 4 * Math.PI); // The
              // constraints
              // for this path.
              // PathConstraints constraints = PathConstraints.unlimitedConstraints(12.0); //
              // You can also use unlimited constraints, only limited by motor torque and
              // nominal battery voltage

              Pose2d fieldPose = Field.conditionallyTransformToOppositeAlliance(pose);

              // this is a point 1m from the end
              Pose2d middlePoint =
                  fieldPose.transformBy(
                      new Transform2d(
                          -1 + RobotConstants.ROBOT_VISION_OFFSET.to(PARTsUnitType.Meter),
                          0,
                          new Rotation2d()));

              Pose2d lastPoint =
                  fieldPose.transformBy(
                      new Transform2d(
                          RobotConstants.ROBOT_VISION_OFFSET.to(PARTsUnitType.Meter),
                          0,
                          new Rotation2d()));
              // Create the path using the waypoints created above
              PathPlannerPath path =
                  new PathPlannerPath(
                      PathPlannerPath.waypointsFromPoses(getPose(), middlePoint, lastPoint),
                      constraints,
                      null, // The ideal starting state, this is only relevant for pre-planned
                      // paths,
                      // so can
                      // be null for on-the-fly paths.
                      new GoalEndState(
                          0.0, fieldPose.getRotation()) // Goal end state. You can set a
                      // holonomic rotation here. If using
                      // a differential drivetrain, the
                      // rotation will have no effect.
                      );

              // Prevent the path from being flipped if the coordinates are already correct
              path.preventFlipping = true;

              return AutoBuilder.followPath(path);
            },
            new HashSet<>(Arrays.asList(this))));
  }

  /**
   * Command to enable rotation to a specific angle while driving (controller used in {@link
   * #drive}).
   *
   * @param angle the angle to rotate to
   * @return the command
   */
  public Command controlledRotateCommand(DoubleSupplier angle, BooleanSupplier condition) {
    return PARTsCommandUtils.setCommandName(
        "PARTsDrivetrain.controlledRotateCommand",
        Commands.run(
                () -> {
                  if (!isControlledRotationEnabled) {
                    thetaController.reset(getPose().getRotation().getRadians());
                  }
                  isControlledRotationEnabled = true;
                  if (!RobotUtils.isBlue()) thetaController.setGoal(angle.getAsDouble() + Math.PI);
                  else thetaController.setGoal(angle.getAsDouble());
                })
            .until(condition)
            .andThen(disableControlledRotation()));
  }

  /**
   * Creates a command that controls the chassis rotation to keep it pointed a specific target
   * location.
   *
   * @param targetPose a supplier for the target pose to point the chassis at
   * @return the command
   */
  public Command targetPoseCommand(Supplier<Pose2d> targetPose, BooleanSupplier condition) {
    return PARTsCommandUtils.setCommandName(
        "PARTsDrivetrain.targetPoseCommand",
        controlledRotateCommand(
            () -> {
              Pose2d target = targetPose.get();
              Transform2d diff = getPose().minus(target);
              Rotation2d rot = new Rotation2d(diff.getX(), diff.getY());
              rot = rot.plus(Rotation2d.kPi);
              return rot.getRadians();
            },
            condition));
  }

  public Command disableControlledRotation() {
    return PARTsCommandUtils.setCommandName(
        "PARTsDrivetrain.disableControlledRotation",
        Commands.runOnce(() -> isControlledRotationEnabled = false));
  }

  public Supplier<Pose2d> supplierGetPose() {
    return this::getPose;
  }

  // endregion

  // region Custom Private Functions

  private void initializeControllers() {

    thetaController =
        new ProfiledPIDController(
            DrivetrainConstants.THETA_P,
            DrivetrainConstants.THETA_I,
            DrivetrainConstants.THETA_D,
            new TrapezoidProfile.Constraints(
                DrivetrainConstants.MAX_AIM_VELOCITY, DrivetrainConstants.MAX_AIM_ACCELERATION));
    thetaController.enableContinuousInput(-Math.PI, Math.PI); // Wrpa from -pi to ip

    xRangeController =
        new ProfiledPIDController(
            DrivetrainConstants.RANGE_X_P,
            DrivetrainConstants.RANGE_I,
            DrivetrainConstants.RANGE_D,
            new TrapezoidProfile.Constraints(
                DrivetrainConstants.MAX_RANGE_VELOCITY,
                DrivetrainConstants.MAX_RANGE_ACCELERATION));
    yRangeController =
        new ProfiledPIDController(
            DrivetrainConstants.RANGE_Y_P,
            DrivetrainConstants.RANGE_I,
            DrivetrainConstants.RANGE_D,
            new TrapezoidProfile.Constraints(
                DrivetrainConstants.MAX_RANGE_VELOCITY,
                DrivetrainConstants.MAX_RANGE_ACCELERATION));
  }

  private void updateAirborneState() {
    double mag = this.gyroIO.getAccelMagnitudeMps2();
    boolean airborneNow = Math.abs(mag) > DrivetrainConstants.AIRBORNE_G_DIFF;
    boolean stableNow = Math.abs(mag) < DrivetrainConstants.STABLE_G_DIFF;

    if (!isAirborne) {
      if (airborneNow) {
        airborneDebounceCycles++;
        if (airborneDebounceCycles >= DrivetrainConstants.AIRBORNE_DEBOUNCE) {
          super.setVisionMeasurementStdDevs(DrivetrainConstants.AIRBORNE_STDEVS);
          isAirborne = true;
          stableDebounceCycles = 0;
          airborneDebounceCycles = 0;
        }
      } else {
        airborneDebounceCycles = 0;
      }
    } else {
      // currently airborne
      if (stableNow) {
        stableDebounceCycles++;
        if (stableDebounceCycles >= DrivetrainConstants.STABLE_DEBOUNCE) {
          super.setVisionMeasurementStdDevs(DrivetrainConstants.STABLE_STDEVS);
          isAirborne = false;
          stableDebounceCycles = 0;
          airborneDebounceCycles = 0;
        }
      } else {
        stableDebounceCycles = 0;
      }
    }
  }

  // endregion

  // region Override Functions
  // endregion

  // region Interface Functions
  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Subsystem");

    builder.addBooleanProperty(".hasDefault", () -> getDefaultCommand() != null, null);
    builder.addStringProperty(
        ".default",
        () -> getDefaultCommand() != null ? getDefaultCommand().getName() : "none",
        null);
    builder.addBooleanProperty(".hasCommand", () -> getCurrentCommand() != null, null);
    builder.addStringProperty(
        ".command",
        () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "none",
        null);
  }

  // endregion

  // region AutoBuilder Functions
  // endregion

  @FunctionalInterface
  public static interface RobotVelocitySupplier {
    public Transform2d get();
  }
}
