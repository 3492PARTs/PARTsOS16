// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.constants.CameraConstants;
import frc.robot.constants.KickerConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.TurretConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Hopper.Hopper;
import frc.robot.subsystems.Hopper.HopperIO;
import frc.robot.subsystems.Hopper.HopperIOTalonFX;
import frc.robot.subsystems.Kicker.Kicker;
import frc.robot.subsystems.Kicker.KickerIO;
import frc.robot.subsystems.Kicker.KickerIOTalonFX;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterIO;
import frc.robot.subsystems.Shooter.ShooterIOTalonFX;
import frc.robot.subsystems.Turret.Turret;
import frc.robot.subsystems.Turret.TurretIO;
import frc.robot.subsystems.Turret.TurretIOTalonFX;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.drive.PARTsDrivetrain;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.parts3492.partslib.input.PARTsButtonBoxController;
import org.parts3492.partslib.input.PARTsCommandController;
import org.parts3492.partslib.input.PARTsController.ControllerType;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final PARTsDrivetrain drive;
  private final Vision vision;
  private final Shooter shooter;
  private final Turret turret;
  private final Kicker kicker;
  private final Hopper hopper;

  // Controller
  private final PARTsCommandController controller =
      new PARTsCommandController(0, ControllerType.XBOX);
  private final PARTsButtonBoxController buttonBoxController = new PARTsButtonBoxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  public static boolean debug = false;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new PARTsDrivetrain(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOLimelight(
                    CameraConstants.LimelightCameras[0].getName(),
                    CameraConstants.LimelightCameras[0].getLocation(),
                    drive::getRotation),
                new VisionIOLimelight(
                    CameraConstants.LimelightCameras[1].getName(),
                    CameraConstants.LimelightCameras[1].getLocation(),
                    drive::getRotation),
                new VisionIOLimelight(
                    CameraConstants.LimelightCameras[2].getName(),
                    CameraConstants.LimelightCameras[2].getLocation(),
                    drive::getRotation),
                new VisionIOLimelight(
                    CameraConstants.LimelightCameras[3].getName(),
                    CameraConstants.LimelightCameras[3].getLocation(),
                    drive::getRotation));
        turret =
            new Turret(
                new TurretIOTalonFX(TurretConstants.TURRET_MOTOR_ID),
                drive::getPose,
                drive::getRobotVelocity);
        shooter =
            new Shooter(
                new ShooterIOTalonFX(
                    ShooterConstants.LEFT_MOTOR_ID, ShooterConstants.RIGHT_MOTOR_ID),
                drive::getPose,
                drive::getRobotVelocity,
                turret::getState);
        kicker = new Kicker(new KickerIOTalonFX(KickerConstants.KICKER_MOTOR_ID));
        hopper = new Hopper(new HopperIOTalonFX());
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new PARTsDrivetrain(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));

        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
        turret = new Turret(new TurretIO() {}, drive::getPose, drive::getRobotVelocity);
        shooter =
            new Shooter(
                new ShooterIO() {}, drive::getPose, drive::getRobotVelocity, turret::getState);
        kicker = new Kicker(new KickerIO() {});
        hopper = new Hopper(new HopperIO() {});
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new PARTsDrivetrain(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
        turret = new Turret(new TurretIO() {}, drive::getPose, drive::getRobotVelocity);
        shooter =
            new Shooter(
                new ShooterIO() {}, drive::getPose, drive::getRobotVelocity, turret::getState);
        kicker = new Kicker(new KickerIO() {});
        hopper = new Hopper(new HopperIO() {});
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
    configureShooterBindings();
    configureCandleBindings();
    configureHopperBindings();
    configureKickerBindings();
    configureTurretBindings();
    configureIntakeBindings();
    configureSuperstructureBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    // Lock to 0° when A button is held
    controller
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> Rotation2d.kZero));

    // Switch to X pattern when X button is pressed
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed
    controller
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));
    controller
        .leftBumper()
        .onTrue(
            Commands.runOnce(
                () -> {
                  shooter.setSpeed(0.25);
                  shooter.setState();
                },
                shooter));
    controller.rightBumper().onTrue(Commands.runOnce(() -> shooter.setSpeed(0), shooter));
  }

  private void configureShooterBindings() {

    /*
     * operatorController.a().and(operatorController.rightBumper())
     * .whileTrue(shooter.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
     * operatorController.b().and(operatorController.rightBumper())
     * .whileTrue(shooter.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
     * operatorController.x().and(operatorController.rightBumper())
     * .whileTrue(shooter.sysIdDynamic(SysIdRoutine.Direction.kForward));
     * operatorController.y().and(operatorController.rightBumper())
     * .whileTrue(shooter.sysIdDynamic(SysIdRoutine.Direction.kReverse));
     */

    // buttonBoxController.lightonTrigger().whileTrue(shooter.setSpeedOffset(100)).onFalse(shooter.setSpeedOffset(0));
    // buttonBoxController.talkonTrigger().whileTrue(shooter.setSpeedOffset(200)).onFalse(shooter.setSpeedOffset(0));
    buttonBoxController.absClickTrigger().onTrue(shooter.setSpeedOffset(() -> 0));
    buttonBoxController
        .absClockwiseTrigger()
        .onTrue(shooter.setSpeedOffset(() -> shooter.getSpeedOffset() + 100));
    buttonBoxController
        .absCounterClockwiseTrigger()
        .onTrue(shooter.setSpeedOffset(() -> shooter.getSpeedOffset() - 100));
  }

  private void configureCandleBindings() {}

  private void configureHopperBindings() {
    /*
     * operatorController.a().and(operatorController.rightBumper())
     * .whileTrue(hopper.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
     * operatorController.b().and(operatorController.rightBumper())
     * .whileTrue(hopper.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
     * operatorController.x().and(operatorController.rightBumper())
     * .whileTrue(hopper.sysIdDynamic(SysIdRoutine.Direction.kForward));
     * operatorController.y().and(operatorController.rightBumper())
     * .whileTrue(hopper.sysIdDynamic(SysIdRoutine.Direction.kReverse));
     * /*driveController.b().onTrue(hopper.roll());
     * driveController.x().onTrue(hopper.idle());
     */

    buttonBoxController
        .negative2Trigger()
        .whileTrue(Commands.parallel(hopper.reverse(), kicker.reverse()))
        .onFalse(Commands.parallel(hopper.idle(), kicker.idle()));
  }

  private void configureKickerBindings() {
    /*
     * operatorController.a().and(operatorController.rightBumper())
     * .whileTrue(kicker.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
     * operatorController.b().and(operatorController.rightBumper())
     * .whileTrue(kicker.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
     * operatorController.x().and(operatorController.rightBumper())
     * .whileTrue(kicker.sysIdDynamic(SysIdRoutine.Direction.kForward));
     * operatorController.y().and(operatorController.rightBumper())
     * .whileTrue(kicker.sysIdDynamic(SysIdRoutine.Direction.kReverse));
     */
  }

  private void configureTurretBindings() {

    /*
     * operatorController.a().and(operatorController.rightBumper())
     * .whileTrue(turret.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
     * operatorController.b().and(operatorController.rightBumper())
     * .whileTrue(turret.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
     * operatorController.x().and(operatorController.rightBumper())
     * .whileTrue(turret.sysIdDynamic(SysIdRoutine.Direction.kForward));
     * operatorController.y().and(operatorController.rightBumper())
     * .whileTrue(turret.sysIdDynamic(SysIdRoutine.Direction.kReverse));
     */

  }

  private void configureIntakeBindings() {
    /*buttonBoxController.positive4Trigger().onTrue(intake.intakeShooting());
    buttonBoxController.negative4Trigger().onTrue(intake.intake());
    buttonBoxController.positive4Trigger().negate().and(buttonBoxController.negative4Trigger().negate())
            .onTrue(intake.idle());
    buttonBoxController.enterTrigger().onTrue(intake.home());
    buttonBoxController.povTrigger0().whileTrue(intake.manualPivot(-0.1)).onFalse(intake.idle());
    buttonBoxController.povTrigger180().whileTrue(intake.manualPivot(0.1)).onFalse(intake.idle());
    buttonBoxController.positive1Trigger().onTrue(intake.zeroArm());
    buttonBoxController.negative1Trigger().onTrue(intake.oneNinetyArm());

    // Intake SysID

     * operatorController.a().and(operatorController.rightBumper())
     * .whileTrue(intake.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
     * operatorController.b().and(operatorController.rightBumper())
     * .whileTrue(intake.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
     * operatorController.x().and(operatorController.rightBumper())
     * .whileTrue(intake.sysIdDynamic(SysIdRoutine.Direction.kForward));
     * operatorController.y().and(operatorController.rightBumper())
     * .whileTrue(intake.sysIdDynamic(SysIdRoutine.Direction.kReverse));
     */

  }

  private void configureSuperstructureBindings() {
    /*buttonBoxController.handleTrigger()
                    .onTrue(superstructure.shoot(buttonBoxController.cruiseTrigger()::getAsBoolean,
                            TurretState.TRACKING_HUB));
            buttonBoxController.enginestartTrigger()
                    .onTrue(superstructure.shoot(buttonBoxController.cruiseTrigger()::getAsBoolean,
                            TurretState.TRACKING_CORNER));
            buttonBoxController.wipeTrigger()
                    .onTrue(superstructure.cornerShoot(buttonBoxController.cruiseTrigger()::getAsBoolean, false));
            buttonBoxController.mapTrigger()
                    .onTrue(superstructure.cornerShoot(buttonBoxController.cruiseTrigger()::getAsBoolean, true));
            buttonBoxController.negative3Trigger().onTrue(superstructure.spew()).onFalse(superstructure.resetCommand());
    */
    // buttonBoxController.escTrigger().whileTrue(superstructure.outpostAuto());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public static boolean isBlue() {
    return false;
  }
}
