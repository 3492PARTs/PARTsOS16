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

    /**
     * lift up pivot arm, roll hopper, roll kicker, shoot. Only happens if turret has valid angle
     */
    public Command shoot(BooleanSupplier end) {

        //kick, stop when shooter not at speed stop, wait till speed up, then repeat
        Command kickAtSpeed = Commands.repeatingSequence(kicker.roll(), new WaitUntilCommand(() -> !shooter.isAtSetpoint()), kicker.idle(), new WaitUntilCommand(shooter::isAtSetpoint));
        Command shoot = Commands.parallel(turret.track(), intake.intakeShooting(), hopper.roll(), shooter.shoot(), kickAtSpeed);
        Command reset = Commands.runOnce(() -> {
                    turret.reset();
                    intake.reset();
                    hopper.reset();
                    kicker.reset();
                    shooter.reset();
                }, hopper, intake, kicker, shooter, turret);
        //shoot, stop when we are at and angle we can't shoot at, wait till we are at a valid angle, then repeat
        Command shootAtAngle = Commands.repeatingSequence(shoot, new WaitUntilCommand(() -> !turret.isValidAngle()), reset, new WaitUntilCommand(turret::isValidAngle));

        // shoot until interrupted
        return PARTsCommandUtils
                .setCommandName("Superstructure.shoot", shootAtAngle.until(end));
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
