package frc.robot.subsystems.Climber;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.constants.ClimberConstants;
import frc.robot.constants.TurretConstants;

public class ClimberPhys extends Climber {
    protected final TalonFX climberMotor;

    public ClimberPhys() {
        super();

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        climberMotor = new TalonFX(ClimberConstants.CLIMBER_MOTOR_ID, TurretConstants.CAN_BUS_NAME);
        climberMotor.getConfigurator().apply(config);
        climberMotor.setNeutralMode(NeutralModeValue.Brake);
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
        partsNT.putDouble("Output", climberMotor.getSupplyCurrent().getValueAsDouble());
        partsNT.putDouble("Current", climberMotor.getSupplyCurrent().getValueAsDouble());
    }

    @Override
    public void log() {
        partsLogger.logDouble("Output", climberMotor.getSupplyCurrent().getValueAsDouble());
        partsLogger.logDouble("Current", climberMotor.getSupplyCurrent().getValueAsDouble());
    }
}
