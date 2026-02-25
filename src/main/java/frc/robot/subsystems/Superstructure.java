package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import org.parts3492.partslib.command.PARTsCommandUtils;
import org.parts3492.partslib.command.PARTsSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
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

    public Superstructure(Hopper hopper, Intake intake, Kicker kicker, Shooter shooter, Turret turret) {
        this.hopper = hopper;
        this.intake = intake;
        this.kicker = kicker;
        this.shooter = shooter;
        this.turret = turret;
    }

    public Command shoot(BooleanSupplier end) {
        Command reset = Commands.parallel(turret.idle(), intake.idle(), hopper.idle(), kicker.idle(), shooter.idle());

        //wait till at shooter at speed, kick, stop and wait when shooter not at speed, stop kicker, repeat
        Command kickAtSpeed = Commands.repeatingSequence(new WaitUntilCommand(shooter::isAtSetpoint), kicker.roll(), new WaitUntilCommand(() -> !shooter.isAtSetpoint()), kicker.idle());
        
        // run all systems and the kick at speed in paralle
        Command shoot = Commands.parallel(turret.track(), intake.intakeShooting(), hopper.roll(), shooter.shoot(), kickAtSpeed);
        
        //wait till we are at a valid angle, shoot, stop when we are at and angle we can't shoot at, then repeat
        Command shootAtAngle = Commands.repeatingSequence(new WaitUntilCommand(turret::isValidAngle), shoot.until(() -> !turret.isValidAngle()), reset);

        // shoot until interrupted
        return PARTsCommandUtils
                .setCommandName("Superstructure.shoot", shootAtAngle.until(end).andThen(reset));
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
