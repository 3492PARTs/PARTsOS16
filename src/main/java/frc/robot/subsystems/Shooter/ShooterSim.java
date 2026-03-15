package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import org.parts3492.partslib.PARTsUnit.PARTsUnitType;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
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
        double moi = 1.0 * Math.PI * ShooterConstants.SHOOTER_WHEEL_THICKNESS.to(PARTsUnitType.Meter) * Math.pow(
            (
                Math.pow(ShooterConstants.SHOOTER_WHEEL_RADIUS.to(PARTsUnitType.Meter), 2) - 
                Math.pow(ShooterConstants.SHOOTER_WHEEL_INNER_RADIUS.to(PARTsUnitType.Meter), 2)
            ), 2);

        talonGearbox = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                        DCMotor.getKrakenX60Foc(2), 0.0001, ShooterConstants.SHOOTER_GEAR_RATIO
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
        
        LinearSystem<N1, N1, N1> plant = LinearSystemId.createFlywheelSystem(talonGearbox.getGearbox(), moi, 1.0);

        shooterSim = new FlywheelSim(plant, talonGearbox.getGearbox(), 0.01);
    }

    @Override
    protected void setSpeed(double speed) {
        leftMotor.set(speed);
    }

    @Override
    protected void setVoltage(double voltage) {
        leftMotor.setVoltage(voltage);
    }

    @Override
    protected double getRPM() {
        return leftMotor.getVelocity().getValueAsDouble() * 60;
    }

    @Override
    protected double getVoltage() {
        return leftMotor.getSupplyVoltage().getValueAsDouble();
    }

    @Override
    public void periodic() {
        super.periodic();
    }

    @Override
    public void simulationPeriodic() {
        leftSim = leftMotor.getSimState();
        rightSim = rightMotor.getSimState();

        leftSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        rightSim.setSupplyVoltage(RobotController.getBatteryVoltage());

        Voltage motorVoltage = leftSim.getMotorVoltageMeasure();

        talonGearbox.setInputVoltage(motorVoltage.in(Volts));
        talonGearbox.update(0.020);

        leftSim.setRawRotorPosition(talonGearbox.getAngularPosition().times(ShooterConstants.SHOOTER_GEAR_RATIO));
        leftSim.setRotorVelocity(talonGearbox.getAngularVelocity().times(ShooterConstants.SHOOTER_GEAR_RATIO));

        rightSim.setRawRotorPosition(talonGearbox.getAngularPosition().times(ShooterConstants.SHOOTER_GEAR_RATIO));
        rightSim.setRotorVelocity(talonGearbox.getAngularVelocity().times(ShooterConstants.SHOOTER_GEAR_RATIO));

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
