package frc.robot.subsystems.Kicker;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.constants.KickerConstants;

public class KickerPhys extends Kicker {
    protected final TalonFX kickerMotor;

    public KickerPhys() {
        super();
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        kickerMotor = new TalonFX(KickerConstants.KICKER_MOTOR_ID, KickerConstants.CAN_BUS_NAME);
        kickerMotor.getConfigurator().apply(config);
        kickerMotor.setNeutralMode(NeutralModeValue.Coast);
    }

    @Override
    public void outputTelemetry() {
        partsNT.putDouble("KickerCurrent", kickerMotor.getSupplyCurrent().getValueAsDouble());

        partsNT.putDouble("KickerOutput", kickerMotor.getStatorCurrent().getValueAsDouble());
    }

    @Override
    public void periodic() {
        super.periodic();
    }

    @Override
    public void log() {
        partsLogger.logDouble("KickerCurrent", kickerMotor.getSupplyCurrent().getValueAsDouble());

        partsLogger.logDouble("KickerOutput", kickerMotor.getStatorCurrent().getValueAsDouble());
    }

    @Override
    protected void setSpeed(double speed) {
        kickerMotor.set(speed);
    }
}
