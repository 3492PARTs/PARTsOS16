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
        return PARTsCommandUtils
                .setCommandName("Superstructure.shoot",
                        Commands.parallel(turret.track(), intake.intakeShooting(), hopper.roll(), kicker.roll(),
                                shooter.shoot()).onlyIf(turret::isValidAngle))
                .andThen(new WaitUntilCommand(() -> !turret.isValidAngle() || end.getAsBoolean()))
                .andThen(Commands.runOnce(() -> {
                    turret.reset();
                    intake.reset();
                    hopper.reset();
                    kicker.reset();
                    shooter.reset();
                }, hopper, intake, kicker, shooter, turret));
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
