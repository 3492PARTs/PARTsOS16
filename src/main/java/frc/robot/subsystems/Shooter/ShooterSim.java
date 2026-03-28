package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import org.parts3492.partslib.PARTsUnit.PARTsUnitType;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.TurretConstants.TurretState;
import frc.robot.subsystems.Drivetrain.PARTsDrivetrain;

public class ShooterSim extends Shooter {

    DCMotorSim talonGearbox;

    protected final TalonFX leftMotor;
    protected final TalonFX rightMotor;

    protected TalonFXSimState leftSim;
    protected TalonFXSimState rightSim;

    FlywheelSim shooterSim;

    public ShooterSim(Supplier <Pose2d> poseSupplier, PARTsDrivetrain drivetrain, Supplier<TurretState> turretSupplierState) {
        super(poseSupplier, drivetrain, turretSupplierState);

        // MOI for flywheel, doesn't account for the shaft.
        // Density * PI * Thickness in Kg * ([Outside Radius ^ 2] - [Inside Radius ^ 2]])^2
        // gearing possibly 1/4 for 4:1 reduction?
        double moi = 0.8 * ShooterConstants.SHOOTER_WHEEL_WEIGHT.to(PARTsUnitType.Kilogram) * 
            (
                Math.pow(ShooterConstants.SHOOTER_WHEEL_RADIUS.to(PARTsUnitType.Meter), 2) + 
                Math.pow(ShooterConstants.SHOOTER_WHEEL_INNER_RADIUS.to(PARTsUnitType.Meter), 2)
            );

        talonGearbox = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                        DCMotor.getKrakenX60Foc(2), moi, ShooterConstants.SHOOTER_GEAR_RATIO
                    ),
                DCMotor.getKrakenX60Foc(2)
        );

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        config.CurrentLimits.SupplyCurrentLimit = 70;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        leftMotor = new TalonFX(ShooterConstants.LEFT_MOTOR_ID, ShooterConstants.CAN_BUS_NAME);
        leftMotor.getConfigurator().apply(config);

        rightMotor = new TalonFX(ShooterConstants.RIGHT_MOTOR_ID, ShooterConstants.CAN_BUS_NAME);

        rightMotor.setControl(new Follower(ShooterConstants.LEFT_MOTOR_ID, MotorAlignmentValue.Opposed));

        leftMotor.setNeutralMode(NeutralModeValue.Coast);
        rightMotor.setNeutralMode(NeutralModeValue.Coast);

        leftSim = leftMotor.getSimState();
        rightSim = rightMotor.getSimState();
        
        LinearSystem<N1, N1, N1> plant = LinearSystemId.createFlywheelSystem(talonGearbox.getGearbox(), moi, ShooterConstants.SHOOTER_GEAR_RATIO);

        //LinearSystem<N1, N1, N1> plant = LinearSystemId.identifyVelocitySystem(ShooterConstants.V, ShooterConstants.A);

        shooterSim = new FlywheelSim(plant, talonGearbox.getGearbox(), 0.01);
    }

    @Override
    protected void setSpeed(double speed) {
        leftMotor.setControl(new VoltageOut(speed * 12.0));
    }

    @Override
    protected void setVoltage(double voltage) {
        leftMotor.setControl(new VoltageOut(voltage));
    }

    @Override
    protected double getRPM() {
        return leftMotor.getVelocity().getValueAsDouble() * 60;
    }

    @Override
    protected double getVoltage() {
        return leftMotor.getMotorVoltage().getValueAsDouble();
    }

    @Override
    public void periodic() {
        super.periodic();
    }


    // In rotations per second
    private double flywheelSimPosition = 0.0;
    @Override
    public void simulationPeriodic() {
        leftSim = leftMotor.getSimState();
        rightSim = rightMotor.getSimState();

        //leftSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        //rightSim.setSupplyVoltage(RobotController.getBatteryVoltage());

        Voltage motorVoltage = leftSim.getMotorVoltageMeasure();

        //talonGearbox.setInputVoltage(motorVoltage.in(Volts));
        //talonGearbox.update(0.02);

        shooterSim.setInput(motorVoltage.in(Volts));
        shooterSim.update(0.02);

        // Default gearbox (No friction I assume since its like ~1800 rpm over the realistic speed :sob:)
        //leftSim.setRawRotorPosition(talonGearbox.getAngularPosition().times(ShooterConstants.SHOOTER_GEAR_RATIO));
        //leftSim.setRotorVelocity(talonGearbox.getAngularVelocity().times(ShooterConstants.SHOOTER_GEAR_RATIO));

        //rightSim.setRawRotorPosition(talonGearbox.getAngularPosition().times(ShooterConstants.SHOOTER_GEAR_RATIO));
        //rightSim.setRotorVelocity(talonGearbox.getAngularVelocity().times(ShooterConstants.SHOOTER_GEAR_RATIO));

        // Flywheel
        flywheelSimPosition += shooterSim.getAngularVelocity().in(RotationsPerSecond) * 0.02;

        leftSim.setRotorVelocity(shooterSim.getAngularVelocity().times(ShooterConstants.SHOOTER_GEAR_RATIO).in(RotationsPerSecond));
        rightSim.setRotorVelocity(shooterSim.getAngularVelocity().times(ShooterConstants.SHOOTER_GEAR_RATIO).in(RotationsPerSecond));

        leftSim.setRawRotorPosition(flywheelSimPosition * ShooterConstants.SHOOTER_GEAR_RATIO);
        rightSim.setRawRotorPosition(flywheelSimPosition * ShooterConstants.SHOOTER_GEAR_RATIO);

        //RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(talonGearbox.getCurrentDrawAmps()));
        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(shooterSim.getCurrentDrawAmps()));

        //shooterSim.setInput(leftSim.getTorqueCurrent(), rightSim.getTorqueCurrent());
        
        //RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(leftSim.getTorqueCurrent(), rightSim.getTorqueCurrent()));
        /*
        shooterSim.setInput(maxSim.getAppliedOutput() * RoboRioSim.getVInVoltage());

        shooterSim.update(0.02);

        double velocityRadPerSec = shooterSim.getAngularVelocityRadPerSec();
        double velocityRPM = Units.radiansPerSecondToRotationsPerMinute(velocityRadPerSec);
        maxSim.iterate(velocityRPM, RoboRioSim.getVInVoltage(), 0.02);

        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(maxSim.getMotorCurrent()));
        */
    }
}
