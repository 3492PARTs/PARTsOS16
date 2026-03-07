package frc.robot.subsystems.Kicker;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.RobotContainer;
import frc.robot.constants.KickerConstants;

public class KickerPhys extends Kicker {
    protected final TalonFX kickerMotor;

    public KickerPhys() {
        super();
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        config.CurrentLimits.SupplyCurrentLimit = 60;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLowerLimit = 30;
        config.CurrentLimits.SupplyCurrentLowerTime = 1.0;

        config.CurrentLimits.StatorCurrentLimit = 100;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        kickerMotor = new TalonFX(KickerConstants.KICKER_MOTOR_ID, KickerConstants.CAN_BUS_NAME);
        kickerMotor.getConfigurator().apply(config);
        kickerMotor.setNeutralMode(NeutralModeValue.Coast);
    }

    @Override
    public void outputTelemetry() {
        super.outputTelemetry();
        partsNT.putDouble("KickerCurrent", kickerMotor.getSupplyCurrent().getValueAsDouble(), RobotContainer.debug || super.debug);

        partsNT.putDouble("KickerOutput", kickerMotor.getStatorCurrent().getValueAsDouble(), RobotContainer.debug || super.debug);
    }

    @Override
    public void periodic() {
        super.periodic();
    }

    @Override
    public void log() {
        partsLogger.logDouble("KickerCurrent", kickerMotor.getSupplyCurrent().getValueAsDouble(), RobotContainer.debug || super.debug);

        partsLogger.logDouble("KickerOutput", kickerMotor.getStatorCurrent().getValueAsDouble(), RobotContainer.debug || super.debug);
    }

    @Override
    protected void setSpeed(double speed) {
        kickerMotor.set(speed);
    }
}
