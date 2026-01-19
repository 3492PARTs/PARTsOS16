package frc.robot.subsystems.Hopper;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.constants.HopperConstants;

public class HopperPhys extends Hopper {
    protected final SparkMax hopperMotor;
    protected final RelativeEncoder hopperEncoder;

    public HopperPhys() {
        super();

        SparkMaxConfig hopperConfig = new SparkMaxConfig();
        hopperConfig.idleMode(IdleMode.kBrake);

        hopperMotor = new SparkMax(HopperConstants.HOPPER_MOTOR_ID, MotorType.kBrushless);
        hopperEncoder = hopperMotor.getEncoder();
        hopperMotor.configure(hopperConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void outputTelemetry() {
        partsNT.putDouble("Output", hopperMotor.getOutputCurrent());
        partsNT.putDouble("Current", hopperMotor.getAppliedOutput());
    }

    @Override
    protected void setSpeed(double speed) {
        hopperMotor.set(speed);
    }

    @Override
    public void periodic() {
        super.periodic();
    }

    @Override
    public void log() {
        partsLogger.logDouble("Output", hopperMotor.getOutputCurrent());
        partsLogger.logDouble("Current", hopperMotor.getAppliedOutput());
    }
}
