package frc.robot.subsystems.Shooter;

import java.util.function.Supplier;

import org.parts3492.partslib.PARTsUnit.PARTsUnitType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.constants.ShooterConstants;

public class ShooterSim extends Shooter {

    DCMotorSim talonGearbox;
    // SparkMax max;
    // SparkMaxSim maxSim;
    // SparkRelativeEncoderSim motorEncoder;
    FlywheelSim shooterSim;

    public ShooterSim(Supplier<Pose2d> poseSupplier) {
        super(poseSupplier);

        // 
        double moi = 1.0 * ShooterConstants.SHOOTER_WEEL_WEIGHT.to(PARTsUnitType.Kilogram)
                * Math.pow(ShooterConstants.SHOOTER_WHEEL_RADIUS.to(PARTsUnitType.Meter), 2);

        talonGearbox = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                        DCMotor.getKrakenX60Foc(2), 0.001, ShooterConstants.SHOOTER_GEAR_RATIO),
                DCMotor.getKrakenX60Foc(1));

        // SparkMaxConfig shooterConfig = new SparkMaxConfig();
        // shooterConfig.idleMode(IdleMode.kCoast);
        // shooterConfig.inverted(true);

        // max = new SparkMax(ShooterConstants.LEFT_MOTOR_ID, MotorType.kBrushless);

        // max.configure(shooterConfig, ResetMode.kResetSafeParameters,
        // PersistMode.kPersistParameters);

        // maxSim = new SparkMaxSim(max, maxGearbox);
        // motorEncoder = maxSim.getRelativeEncoderSim();

        
        LinearSystem<N1, N1, N1> plant = LinearSystemId.createFlywheelSystem(talonGearbox, moi, 1.064);

        // shooterSim = new FlywheelSim(plant, maxGearbox, 0.01);
    }

    @Override
    protected void setSpeed(double speed) {
        max.set(speed);
    }

    @Override
    protected void setVoltage(double voltage) {
        max.setVoltage(voltage);
    }

    @Override
    protected double getRPM() {
        return motorEncoder.getVelocity();
    }

    @Override
    protected double getVoltage() {
        return maxSim.getAppliedOutput() * RoboRioSim.getVInVoltage();
    }

    @Override
    public void periodic() {
        super.periodic();
    }

    @Override
    public void simulationPeriodic() {
        shooterSim.setInput(maxSim.getAppliedOutput() * RoboRioSim.getVInVoltage());

        shooterSim.update(0.02);

        double velocityRadPerSec = shooterSim.getAngularVelocityRadPerSec();
        double velocityRPM = Units.radiansPerSecondToRotationsPerMinute(velocityRadPerSec);
        maxSim.iterate(velocityRPM, RoboRioSim.getVInVoltage(), 0.02);

        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(maxSim.getMotorCurrent()));
    }
}
