package frc.robot.subsystems.Shooter;

import java.util.function.Supplier;

import org.parts3492.partslib.PARTsUnit.PARTsUnitType;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.constants.ShooterConstants;

public class ShooterSim extends Shooter {

    DCMotor maxGearbox;
    //SparkMax max;
    //SparkMaxSim maxSim;
    TalonFX leftMotor;
    TalonFX rightMotor;
    //SparkRelativeEncoderSim motorEncoder;
    FlywheelSim shooterSim;

    public ShooterSim(Supplier <Pose2d> poseSupplier) {
        super(poseSupplier);
        maxGearbox = DCMotor.getKrakenX60Foc(2);

        //SparkMaxConfig shooterConfig = new SparkMaxConfig();
        //shooterConfig.idleMode(IdleMode.kCoast);
        //shooterConfig.inverted(true);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        leftMotor = new TalonFX(ShooterConstants.LEFT_MOTOR_ID, ShooterConstants.CAN_BUS_NAME);
        leftMotor.getConfigurator().apply(config);
        rightMotor = new TalonFX(ShooterConstants.RIGHT_MOTOR_ID, ShooterConstants.CAN_BUS_NAME);

        rightMotor.setControl(new Follower(ShooterConstants.LEFT_MOTOR_ID, MotorAlignmentValue.Opposed));

        leftMotor.setNeutralMode(NeutralModeValue.Coast);
        rightMotor.setNeutralMode(NeutralModeValue.Coast);

        //max = new SparkMax(ShooterConstants.LEFT_MOTOR_ID, MotorType.kBrushless);

        //max.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        //maxSim = new SparkMaxSim(max, maxGearbox);
        //motorEncoder = maxSim.getRelativeEncoderSim();
        
        double moi = 0.6 * ShooterConstants.SHOOTER_WEEL_WEIGHT.to(PARTsUnitType.Kilogram) * Math.pow(ShooterConstants.SHOOTER_WHEEL_RADIUS.to(PARTsUnitType.Meter), 2);
        LinearSystem<N1, N1, N1> plant = LinearSystemId.createFlywheelSystem(maxGearbox, moi, 1.0);

        shooterSim = new FlywheelSim(plant, maxGearbox, 0.01);
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
        return leftMotor.getVelocity().getValueAsDouble();
    }

    @Override
    protected double getVoltage() {
        return leftMotor.getMotorVoltage(true).getValueAsDouble();
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
