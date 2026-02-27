package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import org.parts3492.partslib.command.PARTsCommandUtils;
import org.parts3492.partslib.command.PARTsSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.states.CandleState;
import frc.robot.states.IntakeState;
import frc.robot.states.KickerState;
import frc.robot.states.ShooterState;
import frc.robot.subsystems.Hopper.Hopper;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Kicker.Kicker;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Turret.Turret;

public class Superstructure extends PARTsSubsystem {
    private final Hopper hopper;
    private final Intake intake;
    private final Kicker kicker;
    private final Shooter shooter;
    private final Turret turret;
    private final Candle candle;

    public Superstructure(Hopper hopper, Intake intake, Kicker kicker, Shooter shooter, Turret turret, Candle candle) {
        this.hopper = hopper;
        this.intake = intake;
        this.kicker = kicker;
        this.shooter = shooter;
        this.turret = turret;
        this.candle = candle;
    }

    /**
     * lift up pivot arm, roll hopper, roll kicker, shoot. Only happens if turret has valid angle
     */
    public Command shoot(BooleanSupplier end) {

        return PARTsCommandUtils.setCommandName("Superstructure.shoot",
            
            Commands.sequence(
                // Initial startup
                Commands.parallel(
                    // Start tracking the hub.
                    turret.track(),
                    // Feed the balls into the kicker.
                    hopper.roll(),
                    // Add CANdle shooting state for bot lights.
                    candle.commandAddState(CandleState.SHOOTING)
                )

                .andThen(Commands.repeatingSequence(

                    // Spin up the shooter if the turret is at a valid angle.
                    new ConditionalCommand(
                        shooter.shoot().onlyIf(() -> { return shooter.getState() != ShooterState.SHOOTING; }),
                        shooter.idle().onlyIf(() -> { return shooter.getState() != ShooterState.IDLE; }),
                        turret::isValidAngle
                    ),

                    // Roll the kicker if the shooter is at its setpoint.
                    new ConditionalCommand(
                        kicker.roll().onlyIf(() -> { return kicker.getState() != KickerState.ROLLING; }),
                        kicker.idle().onlyIf(() -> { return kicker.getState() != KickerState.IDLE; }),
                        shooter.atSetpoint()
                    ),

                    new ConditionalCommand(
                        intake.intakeShooting().onlyIf(() -> { return intake.getState() != IntakeState.SHOOTING; }),
                        intake.intakeIdle().onlyIf(() -> { return intake.getState() != IntakeState.IDLE; }),
                        shooter.atSetpoint()
                    ),

                    new ConditionalCommand(
                        candle.commandAddState(CandleState.ACTIVE_SHOOTING)
                            .onlyIf(() -> { return candle.getState() != CandleState.ACTIVE_SHOOTING; }),
                        
                        candle.commandRemoveState(CandleState.ACTIVE_SHOOTING)
                            .onlyIf(() -> { return candle.getState() == CandleState.ACTIVE_SHOOTING; }),
                        shooter.atSetpoint()
                    )
                )
                .until(end)),
                
                // Make sure to cancel and reset if we're forced to end or the turret is not at a valid angle.
                Commands.waitUntil(() -> end.getAsBoolean()).andThen(
                    Commands.runOnce(() -> {
                        turret.reset();
                        intake.reset();
                        hopper.reset();
                        kicker.reset();
                        shooter.reset();
                        candle.removeState(CandleState.SHOOTING);
                    }, hopper, intake, kicker, shooter, turret, candle)
                )
            )
        );
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
