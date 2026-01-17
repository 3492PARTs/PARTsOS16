package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class ShooterPhys extends Shooter {
    protected final SparkMax rightMotor;
    protected final SparkMax leftMotor;

    protected final RelativeEncoder leftEncoder;
    protected final RelativeEncoder rightEncoder;

    public ShooterPhys() {
        super();

        SparkMaxConfig shooterConfig = new SparkMaxConfig();
        shooterConfig.idleMode(IdleMode.kBrake);

        leftMotor = new SparkMax(0, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
        leftEncoder = leftMotor.getEncoder();
        leftMotor.configure(shooterConfig, com.revrobotics.ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
                
        rightMotor = new SparkMax(0, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
        rightEncoder = rightMotor.getEncoder();
        rightMotor.configure(shooterConfig.follow(leftMotor, true), com.revrobotics.ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
    }

    // Make this an abstract function in the main class and define one in each phys and sim.
    // the reason we do this is to create the contract that all shoooters much have a set speed
    // but we leave it up to phys or sim to decide how that should work. 

    @Override
    public void setSpeed(double speed) {
        leftMotor.set(speed);
    }
}
