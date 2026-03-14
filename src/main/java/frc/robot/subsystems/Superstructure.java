package frc.robot.subsystems;

import java.io.IOException;
import java.util.function.BooleanSupplier;

import org.json.simple.parser.ParseException;
import org.parts3492.partslib.command.PARTsCommandUtils;
import org.parts3492.partslib.command.PARTsSubsystem;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.constants.CandleConstants.CandleState;
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

    public Superstructure(Hopper hopper, Intake intake, Kicker kicker, Shooter shooter, Turret turret, Candle candle,
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
                || turretState == TurretState.TRACKING_CORNER);

        Command c = Commands.sequence(
                // Initial startup
                Commands.parallel(
                        // Start tracking the hub.
                        turretState == TurretState.TRACKING_CORNER ? turret.trackCorner() : turret.trackHub(),
                        // Feed the balls into the kicker.
                        hopper.roll(),
                        // Add CANdle shooting state for bot lights.
                        candle.commandAddState(CandleState.SHOOTING))

                        .andThen(Commands.repeatingSequence(

                                // Spin up the shooter if the turret is at a valid angle.
                                new ConditionalCommand(
                                        shooter.shoot().onlyIf(() -> {
                                            return shooter.getState() != ShooterState.SHOOTING;
                                        }),
                                        shooter.idle().onlyIf(() -> {
                                            return shooter.getState() != ShooterState.IDLE;
                                        }),
                                        () -> turret.isValidAngle() && tracking.getAsBoolean()),

                                // Roll the kicker if the shooter is at its setpoint.
                                new ConditionalCommand(
                                        Commands.parallel(kicker.roll(),
                                                candle.commandAddState(CandleState.ACTIVE_SHOOTING)).onlyIf(() -> {
                                                    return kicker.getState() != KickerState.ROLLING;
                                                }),
                                        Commands.parallel(kicker.idle(),
                                                candle.commandRemoveState(CandleState.ACTIVE_SHOOTING)).onlyIf(() -> {
                                                    return kicker.getState() != KickerState.IDLE;
                                                }),
                                        () -> shooter.withinSetpointRange() &&
                                                (shooter.getSetpoint().getAsDouble() > 0)
                                                && turret.isValidAngle() &&
                                                turret.withinSetpointRange() &&
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
                        shooter.manualShoot(), // make a manual state command in the shooter and call
                        candle.commandAddState(CandleState.SHOOTING))

                        .andThen(Commands.repeatingSequence(

                                // Roll the kicker if the shooter is at its setpoint.
                                new ConditionalCommand(
                                        Commands.parallel(kicker.roll(),
                                                candle.commandAddState(CandleState.ACTIVE_SHOOTING)).onlyIf(() -> {
                                                    return kicker.getState() != KickerState.ROLLING;
                                                }),
                                        Commands.parallel(kicker.idle(),
                                                candle.commandRemoveState(CandleState.ACTIVE_SHOOTING)).onlyIf(() -> {
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
                            AutoBuilder.followPath(PathPlannerPath.fromPathFile(left ? "LeftTrenchToCenter" : "RightTrenchToCenter")),
                            Commands.sequence(new WaitCommand(.4), intake.intake())),
                    AutoBuilder.followPath(PathPlannerPath.fromPathFile(left ? "LeftCenterCollectBalls" : "RightCenterCollectBalls")),
                    AutoBuilder.followPath(PathPlannerPath.fromPathFile(left ? "LeftCenterToTrench" : "RightCenterToTrench")),
                    Commands.parallel(shoot(() -> false, TurretState.TRACKING_HUB),
                            Commands.sequence(new WaitCommand(1), intake.intakeShooting())));
        } catch (FileVersionException | IOException | ParseException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
        return PARTsCommandUtils.setCommandName("Superstructure.trenchAuto", c);
    }

    public Command outpostAuto() {
        Command c = new WaitCommand(0);
        try {
            c = Commands.sequence(
                    Commands.parallel(
                            AutoBuilder.followPath(PathPlannerPath.fromPathFile("RightRampToOutpost")),
                            intake.intake()),
                    Commands.parallel(shoot(() -> false, TurretState.TRACKING_HUB),
                            Commands.sequence(new WaitCommand(3), intake.intakeShooting())));
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
