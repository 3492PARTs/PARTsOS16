package frc.robot.subsystems.Kicker;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import frc.robot.constants.KickerConstants;

public class KickerPhys extends Kicker {
    protected final SparkMax kickerMotor;

    protected final RelativeEncoder kickerEncoder;

    public KickerPhys() {
        super();

        SparkMaxConfig kickerConfig = new SparkMaxConfig();
        kickerConfig.idleMode(IdleMode.kCoast);

        kickerMotor = new SparkMax(KickerConstants.KICKER_MOTOR_ID,
                MotorType.kBrushless);
        kickerEncoder = kickerMotor.getEncoder();
        kickerMotor.configure(kickerConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
    }

    @Override
    public void outputTelemetry() {
        partsNT.putDouble("KickerCurrent", kickerMotor.getOutputCurrent());

        partsNT.putDouble("KickerOutput", kickerMotor.getAppliedOutput());
    }

    @Override
    protected void setSpeed(double speed) {
        kickerMotor.set(speed);
    }

    @Override
    public void periodic() {
        super.periodic();
    }

    @Override
    public void log() {
        partsLogger.logDouble("KickerCurrent", kickerMotor.getOutputCurrent());

        partsLogger.logDouble("KickerOutput", kickerMotor.getAppliedOutput());
    }
}
