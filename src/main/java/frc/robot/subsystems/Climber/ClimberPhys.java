package frc.robot.subsystems.Climber;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.constants.ClimberConstants;

public class ClimberPhys extends Climber {
    protected final SparkMax climberMotor;
    protected final RelativeEncoder climberEncoder;

    public ClimberPhys() {
        super();

        SparkMaxConfig climberConfig = new SparkMaxConfig();
        climberConfig.idleMode(IdleMode.kBrake);
                
        climberMotor = new SparkMax(ClimberConstants.CLIMBER_MOTOR_ID, MotorType.kBrushless);
        climberEncoder = climberMotor.getEncoder();
        climberMotor.configure(climberConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    protected void setSpeed(double speed) {
        climberMotor.set(speed);
    }

    @Override
    public void periodic() {
        super.periodic();
    }

    @Override
    public void outputTelemetry() {
        partsNT.putDouble("Output", climberMotor.getOutputCurrent());
        partsNT.putDouble("Current", climberMotor.getAppliedOutput());
    }

    @Override
    public void log() {
        partsLogger.logDouble("Output", climberMotor.getOutputCurrent());
        partsLogger.logDouble("Current", climberMotor.getAppliedOutput());
    }
}
