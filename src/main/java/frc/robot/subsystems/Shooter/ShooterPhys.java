package frc.robot.subsystems.Shooter;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.PowerDistribution;
import frc.robot.constants.ShooterConstants;

public class ShooterPhys extends Shooter {
    protected final SparkMax rightMotor;
    protected final SparkMax leftMotor;

    protected final RelativeEncoder leftEncoder;
    protected final RelativeEncoder rightEncoder;

    public ShooterPhys() {
        super();

        SparkMaxConfig shooterConfig = new SparkMaxConfig();
        shooterConfig.idleMode(IdleMode.kCoast);

        // This is the left motor.
        // It is the leader in the leader-follower configuration.
        leftMotor = new SparkMax(ShooterConstants.LEFT_MOTOR_ID, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
        leftEncoder = leftMotor.getEncoder();
        leftMotor.configure(shooterConfig, com.revrobotics.ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        
        // This is the right motor.
        // It is the follower in the leader-follower configuration.
        rightMotor = new SparkMax(ShooterConstants.RIGHT_MOTOR_ID, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
        rightEncoder = rightMotor.getEncoder();
        rightMotor.configure(shooterConfig.follow(leftMotor, true), com.revrobotics.ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
    }

    @Override
    public void outputTelemetry() {
        super.outputTelemetry();
        partsNT.putDouble("Current/Left", leftMotor.getOutputCurrent());
        partsNT.putDouble("Current/Right", rightMotor.getOutputCurrent());

        partsNT.putDouble("Output/Left", leftMotor.getAppliedOutput());
        partsNT.putDouble("Output/Right", rightMotor.getAppliedOutput());
    }

    @Override
    protected void setSpeed(double speed) {
        leftMotor.set(speed);
    }

    @Override
    protected double getRPM() {
        return leftEncoder.getVelocity();
    }

    @Override
    protected void setVoltage(double voltage) {
        leftMotor.setVoltage(voltage);
    }

    @Override
    protected double getVoltage() {
        return leftMotor.getBusVoltage();
    }

    @Override
    public void periodic() {
        super.periodic();
    }

    @Override
    public void log() {
        super.log();
        partsLogger.logDouble("Current/Left", leftMotor.getOutputCurrent());
        partsLogger.logDouble("Current/Right", rightMotor.getOutputCurrent());

        partsLogger.logDouble("Output/Left", leftMotor.getAppliedOutput());
        partsLogger.logDouble("Output/Right", rightMotor.getAppliedOutput());
    }
}
