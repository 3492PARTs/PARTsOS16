package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import org.parts3492.partslib.command.PARTsCommandUtils;
import org.parts3492.partslib.command.PARTsSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.states.CandleState;
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

    public Command shoot(BooleanSupplier end) {
        //wait till at shooter at speed, kick, stop and wait when shooter not at speed, stop kicker, repeat
        Command kickAtSpeed = Commands.repeatingSequence(new WaitUntilCommand(shooter::isAtSetpoint), kicker.roll(), candle.addStateCmd(CandleState.SHOOT_CMD_SHOOTING), new WaitUntilCommand(() -> !shooter.isAtSetpoint()), kicker.idle(), candle.removeStateCmd(CandleState.SHOOT_CMD_SHOOTING));
        
        // run all systems and the kick at speed in parallel
        Command shoot = Commands.parallel(turret.track(), intake.intakeShooting(), hopper.roll(), shooter.shoot(), kickAtSpeed);
        
        //wait till we are at a valid angle, shoot, stop when we are at and angle we can't shoot at, then repeat
        Command shootAtAngle = Commands.repeatingSequence(new WaitUntilCommand(turret::isValidAngle), shoot.until(() -> !turret.isValidAngle()), resetSubsystems());
        
        Command c = candle.addStateCmd(CandleState.SHOOT_CMD_ACTIVE).andThen(shootAtAngle.until(end)).andThen(resetSubsystems()).andThen(candle.removeStateCmd(CandleState.SHOOT_CMD_SHOOTING)).andThen(candle.removeStateCmd(CandleState.SHOOT_CMD_ACTIVE));
        c.addRequirements(this);

        // shoot until interrupted
        return PARTsCommandUtils
                .setCommandName("Superstructure.shoot", c);
    }

    private Command resetSubsystems() {
        return Commands.parallel(turret.idle(), intake.idle(), hopper.idle(), kicker.idle(), shooter.idle());
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
