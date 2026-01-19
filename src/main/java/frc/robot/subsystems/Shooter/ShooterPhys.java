package frc.robot.subsystems.Shooter;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

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

        leftMotor = new SparkMax(ShooterConstants.LEFT_MOTOR_ID, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
        leftEncoder = leftMotor.getEncoder();
        leftMotor.configure(shooterConfig, com.revrobotics.ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
                
        rightMotor = new SparkMax(ShooterConstants.RIGHT_MOTOR_ID, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
        rightEncoder = rightMotor.getEncoder();
        rightMotor.configure(shooterConfig.follow(leftMotor, true), com.revrobotics.ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
    }

    @Override
    protected void setSpeed(double speed) {
        leftMotor.set(speed);
    }

    @Override
    public void periodic() {
        super.periodic();
    }
}
