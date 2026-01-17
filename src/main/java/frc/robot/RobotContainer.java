// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.generated.TunerConstants;
import frc.robot.states.CandleState;
import frc.robot.subsystems.Candle;
import frc.robot.subsystems.LimelightVision;
import frc.robot.subsystems.LimelightVision.MegaTagMode;
import frc.robot.subsystems.Drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.Drivetrain.PARTsDrivetrain;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterPhys;
import frc.robot.subsystems.Shooter.ShooterSim;
import frc.robot.util.PARTs.Classes.PARTsButtonBoxController;
import frc.robot.util.PARTs.Classes.PARTsCommandController;
import frc.robot.util.PARTs.Classes.PARTsController.ControllerType;
import frc.robot.util.PARTs.Classes.PARTsDashboard;
import frc.robot.util.PARTs.Classes.PARTsNT;
import frc.robot.util.PARTs.Interfaces.IPARTsSubsystem;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top
                                                                                        // speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second
                                                                                      // max angular velocity

    private boolean visionAlignActive = true;
    private BooleanSupplier visionAlignActiveBooleanSupplier = () -> visionAlignActive;

    private final PARTsCommandController driveController = new PARTsCommandController(0, ControllerType.DS5);
    private final PARTsCommandController operatorController = new PARTsCommandController(1, ControllerType.XBOX);
    private final PARTsButtonBoxController buttonBoxController = new PARTsButtonBoxController(2);

    private PARTsNT partsNT = new PARTsNT("RobotContainer");

     private SendableChooser<Command> autoChooser;

    /* Subsystems */

    public final PARTsDrivetrain drivetrain = new PARTsDrivetrain(
            TunerConstants.DrivetrainConstants,
            TunerConstants.FrontLeft, TunerConstants.FrontRight, TunerConstants.BackLeft,
            TunerConstants.BackRight);

    private final LimelightVision vision = new LimelightVision(drivetrain.supplierGetPose(),
            drivetrain.biConsumerAddVisionMeasurement(), drivetrain.consumerSetVisionMeasurementStdDevs(),
            drivetrain.consumerResetPose());

    public final Candle candle = new Candle();

    private final Shooter shooter = Robot.isReal() ? new ShooterPhys() : new ShooterSim();

     private final ArrayList<IPARTsSubsystem> subsystems = new ArrayList<IPARTsSubsystem>(
                        Arrays.asList(candle, drivetrain, vision, shooter));


    /* End Subsystems */

    public RobotContainer() {
        configureDrivetrainBindings();
        configureCandleBindings();
        configureShooterBindings();

        // partsNT.putSmartDashboardSendable("field", Field.FIELD2D);
    }

    /* Configs */

    private void configureDrivetrainBindings() {

        // Drivetrain will execute this command periodically
        drivetrain.setDefaultCommand(drivetrain.commandJoystickDrive(driveController));

        // fine grain controls
        // driveController.rightBumper().onTrue(Commands.runOnce(() -> fineGrainDrive =
        // !fineGrainDrive));

        // new Trigger(() -> fineGrainDrive)
        // .onTrue(Commands.runOnce(() ->
        // candle.addState(CandleState.FINE_GRAIN_DRIVE)))
        // .onFalse(Commands.runOnce(() ->
        // candle.removeState(CandleState.FINE_GRAIN_DRIVE)));

        // brakes swerve, puts modules into x configuration
        driveController.a().whileTrue(drivetrain.commandBrake());

        // manual module direction control
        driveController.b().whileTrue(drivetrain.commandPointWheels(driveController));

        // reset the field-centric heading on left bumper press
        driveController.leftBumper().onTrue(drivetrain.commandSeedFieldCentric());

        /*
         * if (RobotConstants.DEBUGGING) {
         * 
         * //driveController.rightTrigger()
         * // .whileTrue(drivetrain.commandPathOnTheFly(
         * // Field.getTag(12).getLocation().toPose2d()));
         * driveController.rightTrigger()
         * .whileTrue(Reef.commandIntakeScoreIntake(drivetrain, coral, elevator));
         * driveController.leftTrigger()
         * .whileTrue(vision.commandMegaTagMode(MegaTagMode.MEGATAG2));
         * }
         */

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        /*
         * 
         * driveController.back().and(driveController.y()).whileTrue(drivetrain.
         * sysIdDynamic(Direction.kForward));
         * driveController.back().and(driveController.x()).whileTrue(drivetrain.
         * sysIdDynamic(Direction.kReverse));
         * driveController.start().and(driveController.y()).whileTrue(drivetrain.
         * sysIdQuasistatic(Direction.kForward));
         * driveController.start().and(driveController.x()).whileTrue(drivetrain.
         * sysIdQuasistatic(Direction.kReverse));
         */

    }

    private void configureShooterBindings() {
        driveController.a().onTrue(Commands.runOnce(() -> shooter.setSpeed(0.25)));
        driveController.b().onTrue(Commands.runOnce(() -> shooter.setSpeed(0)));
    }

    private void configureCandleBindings() {

    }

    public void configureAutonomousCommands() {
        //autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    /* End Configs */

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }



    public void outputTelemetry() {
        subsystems.forEach(s -> s.outputTelemetry());
        partsNT.putBoolean("Vision Mode", visionAlignActive);
        partsNT.putDouble("Battery Voltage", RobotController.getBatteryVoltage());
    }

    public void stop() {
        subsystems.forEach(s -> s.stop());
    }

    public void log() {
        subsystems.forEach(s -> s.log());
    }

    public void setCandleDisabledState() {
        candle.removeState(CandleState.IDLE);
        candle.addState(CandleState.DISABLED);
    }

    public void setIdleCandleState() {
        candle.addState(CandleState.IDLE);
        candle.removeState(CandleState.DISABLED);
    }

    public void constructDashboard() {
        PARTsDashboard.setSubsystems(subsystems);
        PARTsDashboard.setCommandScheduler();
    }

    public void resetStartPose() {
        drivetrain.seedFieldCentric();
    }

    public void setMegaTagMode(MegaTagMode mode) {
        vision.setMegaTagMode(mode);
    }
}
