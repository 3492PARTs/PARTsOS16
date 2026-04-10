package frc.robot.subsystems;

import java.io.IOException;
import java.util.function.BooleanSupplier;

import org.json.simple.parser.ParseException;
import org.parts3492.partslib.command.PARTsCommandUtils;
import org.parts3492.partslib.command.PARTsSubsystem;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.constants.CandleConstants.CandleState;
import frc.robot.constants.HopperConstants.HopperState;
import frc.robot.constants.KickerConstants.KickerState;
import frc.robot.constants.ShooterConstants.ShooterState;
import frc.robot.constants.TurretConstants.TurretState;
import frc.robot.subsystems.Drivetrain.PARTsDrivetrain;
import frc.robot.subsystems.Hopper.Hopper;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Kicker.Kicker;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Turret.Turret;
import frc.robot.util.Field;

public class Superstructure extends PARTsSubsystem {
        private final Hopper hopper;
        private final Intake intake;
        private final Kicker kicker;
        private final Shooter shooter;
        private final Turret turret;
        private final Candle candle;
        private final PARTsDrivetrain drivetrain;

        boolean shoot = false;

        BooleanSupplier shootBooleanSupplier = () -> shoot;
        BooleanConsumer shootBooleanConsumer = (b1) -> shoot = b1;

        public Superstructure(Hopper hopper, Intake intake, Kicker kicker, Shooter shooter, Turret turret,
                        Candle candle,
                        PARTsDrivetrain drivetrain) {
                this.hopper = hopper;
                this.intake = intake;
                this.kicker = kicker;
                this.shooter = shooter;
                this.turret = turret;
                this.candle = candle;
                this.drivetrain = drivetrain;
        }

        /**
         * lift up pivot arm, roll hopper, roll kicker, shoot. Only happens if turret
         * has valid angle
         */
        public Command shoot(BooleanSupplier end, TurretState turretState) {
                BooleanSupplier tracking = () -> ((turretState == TurretState.TRACKING_HUB
                                && Field.isInAllianceZone(drivetrain.getPose()))
                                || (turretState == TurretState.TRACKING_CORNER
                                                && !Field.isInAllianceZone(drivetrain.getPose())));

                Command c = Commands.sequence(
                                // Initial startup
                                Commands.parallel(
                                                // Start tracking the hub.
                                                turretState == TurretState.TRACKING_CORNER ? turret.trackCorner()
                                                                : turret.trackHub(),
                                                // Feed the balls into the kicker.

                                                // Add CANdle shooting state for bot lights.
                                                candle.commandAddState(CandleState.SHOOTING))

                                                .andThen(Commands.repeatingSequence(
                                                                hopper.roll().onlyIf(() -> hopper
                                                                                .getState() != HopperState.REVERSE
                                                                                && hopper.getState() != HopperState.ROLLING),

                                                                // Spin up the shooter if the turret is at a valid
                                                                // angle.
                                                                new ConditionalCommand(
                                                                                shooter.shoot().onlyIf(() -> {
                                                                                        return shooter.getState() != ShooterState.SHOOTING;
                                                                                }),
                                                                                shooter.idle().onlyIf(() -> {
                                                                                        return shooter.getState() != ShooterState.IDLE;
                                                                                }),
                                                                                () -> turret.isValidAngle() && tracking
                                                                                                .getAsBoolean()),

                                                                // Roll the kicker if the shooter is at its setpoint.
                                                                new ConditionalCommand(
                                                                                Commands.parallel(kicker.roll(),
                                                                                                candle.commandAddState(
                                                                                                                CandleState.ACTIVE_SHOOTING))
                                                                                                .onlyIf(() -> {
                                                                                                        return kicker.getState() != KickerState.ROLLING;
                                                                                                }),
                                                                                Commands.parallel(kicker.idle(),
                                                                                                candle.commandRemoveState(
                                                                                                                CandleState.ACTIVE_SHOOTING))
                                                                                                .onlyIf(() -> {
                                                                                                        return kicker.getState() != KickerState.IDLE;
                                                                                                }),
                                                                                () -> shooter.withinSetpointRange() &&
                                                                                                (shooter.getSetpoint()
                                                                                                                .getAsDouble() > 0)
                                                                                                && turret.isValidAngle()
                                                                                                &&
                                                                                                turret.withinSetpointRange()
                                                                                                &&
                                                                                                tracking.getAsBoolean()))
                                                                .until(end)),

                                // Make sure to cancel and reset if we're forced to end or the turret is not at
                                // a valid angle.
                                Commands.waitUntil(() -> end.getAsBoolean()).andThen(
                                                Commands.runOnce(() -> {
                                                        turret.reset();
                                                        intake.reset();
                                                        hopper.reset();
                                                        kicker.reset();
                                                        shooter.reset();
                                                        candle.removeState(CandleState.SHOOTING);
                                                        candle.removeState(CandleState.ACTIVE_SHOOTING);
                                                })));
                c.addRequirements(this);
                return PARTsCommandUtils.setCommandName("Superstructure.shoot", c);
        }

        public Command cornerShoot(BooleanSupplier end, boolean right) {
                Command c = Commands.sequence(
                                // Initial startup
                                Commands.parallel(
                                                right ? turret.rightCorner() : turret.leftCorner(),
                                                hopper.roll(),
                                                shooter.manualShoot(), // make a manual state command in the shooter and
                                                                       // call
                                                candle.commandAddState(CandleState.SHOOTING))

                                                .andThen(Commands.repeatingSequence(

                                                                // Roll the kicker if the shooter is at its setpoint.
                                                                new ConditionalCommand(
                                                                                Commands.parallel(kicker.roll(),
                                                                                                candle.commandAddState(
                                                                                                                CandleState.ACTIVE_SHOOTING))
                                                                                                .onlyIf(() -> {
                                                                                                        return kicker.getState() != KickerState.ROLLING;
                                                                                                }),
                                                                                Commands.parallel(kicker.idle(),
                                                                                                candle.commandRemoveState(
                                                                                                                CandleState.ACTIVE_SHOOTING))
                                                                                                .onlyIf(() -> {
                                                                                                        return kicker.getState() != KickerState.IDLE;
                                                                                                }),
                                                                                () -> turret.atSetpoint()))
                                                                .until(end)),

                                // Make sure to cancel and reset if we're forced to end or the turret is not at
                                // a valid angle.
                                Commands.waitUntil(() -> end.getAsBoolean()).andThen(
                                                Commands.runOnce(() -> {
                                                        turret.reset();
                                                        intake.reset();
                                                        hopper.reset();
                                                        kicker.reset();
                                                        shooter.reset();
                                                        candle.removeState(CandleState.SHOOTING);
                                                        candle.removeState(CandleState.ACTIVE_SHOOTING);
                                                })));

                c.addRequirements(this);
                return PARTsCommandUtils.setCommandName("Superstructure.shoot", c);
        }

        public Command trenchAuto(boolean left) {
                Command c = new WaitCommand(0);
                try {
                        c = Commands.sequence(
                                        Commands.parallel(
                                                        AutoBuilder.followPath(
                                                                        PathPlannerPath.fromPathFile(left
                                                                                        ? "LeftTrenchToCenter"
                                                                                        : "RightTrenchToCenter")),
                                                        Commands.sequence(new WaitCommand(.5), intake.intake())),
                                        AutoBuilder.followPath(
                                                        PathPlannerPath.fromPathFile(left ? "LeftCenterCollectBalls"
                                                                        : "RightCenterCollectBalls")),
                                        AutoBuilder.followPath(
                                                        PathPlannerPath.fromPathFile(left ? "LeftCenterToTrench1"
                                                                        : "RightCenterToTrench1")),
                                        AutoBuilder.followPath(
                                                        PathPlannerPath.fromPathFile(left ? "LeftCenterToTrench2"
                                                                        : "RightCenterToTrench2")),
                                        intake.idle(),
                                        Commands.parallel(shoot(() -> false, TurretState.TRACKING_HUB),
                                                        Commands.sequence(new WaitCommand(2),
                                                                        intake.intakeShooting())));
                } catch (FileVersionException | IOException | ParseException e) {
                        // TODO Auto-generated catch block
                        e.printStackTrace();
                }
                return PARTsCommandUtils.setCommandName("Superstructure.trenchAuto", c);
        }

        /*
         * public Command rightTrenchOutpostAuto() {
         * Command c = new WaitCommand(0);
         * try {
         * c = Commands.sequence(
         * Commands.parallel(
         * AutoBuilder.followPath(PathPlannerPath
         * .fromPathFile("RightTrenchToCenter")),
         * Commands.sequence(new WaitCommand(.5), intake.intake())),
         * 
         * AutoBuilder.followPath(PathPlannerPath
         * .fromPathFile("RightCenterCollectBalls")),
         * 
         * AutoBuilder.followPath(PathPlannerPath.fromPathFile("RightCenterToTrench1")),
         * Commands.parallel(shoot(() -> false, TurretState.TRACKING_HUB),
         * AutoBuilder.followPath(PathPlannerPath
         * .fromPathFile("RightTrenchForwardToOutpost")),
         * Commands.sequence(new WaitCommand(9),
         * intake.intakeShooting())));
         * } catch (FileVersionException | IOException | ParseException e) {
         * // TODO Auto-generated catch block
         * e.printStackTrace();
         * }
         * return
         * PARTsCommandUtils.setCommandName("Superstructure.rightTrenchOutpostAuto", c);
         * }
         */

        public Command outpostAuto() {
                Command c = new WaitCommand(0);
                try {
                        c = Commands.sequence(
                                        Commands.parallel(
                                                        AutoBuilder.followPath(PathPlannerPath
                                                                        .fromPathFile("RightRampToOutpost")),
                                                        intake.intake()),
                                        Commands.parallel(shoot(() -> false, TurretState.TRACKING_HUB),
                                                        Commands.sequence(new WaitCommand(3),
                                                                        intake.intakeShooting())));
                } catch (FileVersionException | IOException | ParseException e) {
                        // TODO Auto-generated catch block
                        e.printStackTrace();
                }
                return PARTsCommandUtils.setCommandName("Superstructure.trenchAuto", c);
        }

        public Command rightTrenchOutpostAuto() {

                Command c = new WaitCommand(0);
                try {
                        // find way to stop shooting command. needs to take in supplier that we change
                        // to true when we are done

                        c = Commands.sequence(
                                        Commands.parallel(
                                                        AutoBuilder.followPath(PathPlannerPath
                                                                        .fromPathFile("CollectRightMiddle")),
                                                        Commands.sequence(new WaitCommand(1), intake.intake())),

                                        AutoBuilder.followPath(PathPlannerPath.fromPathFile("RightRampToRightTrench")),

                                        Commands.parallel(shoot(shootBooleanSupplier, TurretState.TRACKING_HUB),
                                                        AutoBuilder.followPath(PathPlannerPath
                                                                        .fromPathFile("RightTrenchToOutpost")),
                                                        Commands.sequence(new WaitCommand(2.25),
                                                                        intake.intakeShooting())),
                                        // turn off intake shooting
                                        intake.idle(),
                                        Commands.runOnce(() -> shootBooleanConsumer.accept(true), this),
                                        AutoBuilder.followPath(PathPlannerPath.fromPathFile("OutpostToRightTrench")),

                                        Commands.parallel(
                                                        AutoBuilder.followPath(PathPlannerPath
                                                                        .fromPathFile("CollectRightMiddle")),
                                                        Commands.sequence(new WaitCommand(1)), intake.intake()),

                                        AutoBuilder.followPath(PathPlannerPath.fromPathFile("RightRampToRightTrench")),
                                        // intake off
                                        intake.idle());

                } catch (FileVersionException | IOException | ParseException e) {
                        e.printStackTrace();
                }
                return PARTsCommandUtils.setCommandName("Superstructure.trenchAuto", c);
        }

        public Command leftTrenchAuto() {

                Command c = new WaitCommand(0);
                try {
                        c = Commands.sequence(
                                        Commands.parallel(
                                                        AutoBuilder.followPath(PathPlannerPath
                                                                        .fromPathFile("CollectLeftMiddle"))),
                                        Commands.sequence(new WaitCommand(.7), intake.intake()),

                                        Commands.parallel(shoot(shootBooleanSupplier, TurretState.TRACKING_HUB),
                                                        AutoBuilder.followPath(PathPlannerPath
                                                                        .fromPathFile("LeftRampToLeftTrench")),
                                                        Commands.sequence(new WaitCommand(.2),
                                                                        intake.intakeShooting())),

                                        intake.intake(),
                                        Commands.runOnce(() -> shootBooleanConsumer.accept(true), this),

                                        AutoBuilder.followPath(
                                                        PathPlannerPath.fromPathFile("CollectLeftMiddleAfterShoot")),

                                        Commands.parallel(shoot(shootBooleanSupplier, TurretState.TRACKING_HUB),
                                                        AutoBuilder.followPath(PathPlannerPath
                                                                        .fromPathFile("LeftRampToLeftTrench")),
                                                        Commands.sequence(new WaitCommand(.2), intake.intakeShooting()))

                        );
                } catch (FileVersionException | IOException | ParseException e) {
                        e.printStackTrace();
                }
                return PARTsCommandUtils.setCommandName("Superstructure.trenchAuto", c);
        }

                public Command strategicRightTrenchAuto() {
                Command c = new WaitCommand(0);
                try {
                        c = Commands.sequence(
                                        Commands.parallel(shoot(shootBooleanSupplier, TurretState.TRACKING_HUB),
                                                        AutoBuilder.followPath(PathPlannerPath
                                                                        .fromPathFile("RampShoot")),
                                                        intake.intakeShooting()),

                                        Commands.runOnce(() -> shootBooleanConsumer.accept(true), this),

                                        new WaitCommand(7),
                                        /* insert however many seconds needed to wait for alliance team to do trench auto */

                                        Commands.parallel(
                                                AutoBuilder.followPath(PathPlannerPath
                                                        .fromPathFile("RampToCollectRightMiddle")),
                                                Commands.sequence(new WaitCommand(1), intake.intake())),

                                        Commands.parallel(Commands.parallel(shoot(shootBooleanSupplier, TurretState.TRACKING_HUB),
                                                        AutoBuilder.followPath(PathPlannerPath
                                                                        .fromPathFile("RightRampToRightTrench")),
                                                        Commands.sequence(new WaitCommand(.2), intake.intakeShooting())))
                                );
                } catch (FileVersionException | IOException | ParseException e) {
                        // TODO Auto-generated catch block
                        e.printStackTrace();
                }
                return PARTsCommandUtils.setCommandName("Superstructure.trenchAuto", c);
        }

        @Override
        public void outputTelemetry() {
        }

        @Override
        public void stop() {
        }

        @Override
        public void reset() {
        }

        @Override
        public void log() {
        }
}
