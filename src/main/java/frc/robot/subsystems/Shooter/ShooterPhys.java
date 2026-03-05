package frc.robot.subsystems.Shooter;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.constants.ShooterConstants;

public class ShooterPhys extends Shooter {
    protected final TalonFX leftMotor;
    protected final TalonFX rightMotor;

    public ShooterPhys(Supplier <Pose2d> poseSupplier) {
        super(poseSupplier);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        config.CurrentLimits.SupplyCurrentLimit = 70;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        leftMotor = new TalonFX(ShooterConstants.LEFT_MOTOR_ID, ShooterConstants.CAN_BUS_NAME);
        leftMotor.getConfigurator().apply(config);

        rightMotor = new TalonFX(ShooterConstants.RIGHT_MOTOR_ID, ShooterConstants.CAN_BUS_NAME);

        rightMotor.setControl(new Follower(ShooterConstants.LEFT_MOTOR_ID, MotorAlignmentValue.Opposed));

        leftMotor.setNeutralMode(NeutralModeValue.Coast);
        rightMotor.setNeutralMode(NeutralModeValue.Coast);
    }

    @Override
    public void outputTelemetry() {
        super.outputTelemetry();
        partsNT.putDouble("Current/Left", leftMotor.getSupplyCurrent().getValueAsDouble());
        partsNT.putDouble("Current/Right", rightMotor.getSupplyCurrent().getValueAsDouble());

        partsNT.putDouble("Output/Left", leftMotor.getStatorCurrent().getValueAsDouble());
        partsNT.putDouble("Output/Right", rightMotor.getStatorCurrent().getValueAsDouble());
    }

    @Override
    public void periodic() {
        super.periodic();
    }

    @Override
    public void log() {
        super.log();
        partsLogger.logDouble("Current/Left", leftMotor.getSupplyCurrent().getValueAsDouble());
        partsLogger.logDouble("Current/Right", rightMotor.getSupplyCurrent().getValueAsDouble());

        partsLogger.logDouble("Output/Left", leftMotor.getStatorCurrent().getValueAsDouble());
        partsLogger.logDouble("Output/Right", rightMotor.getStatorCurrent().getValueAsDouble());
    }

    @Override
    protected void setSpeed(double speed) {
        leftMotor.set(speed);
    }

    @Override
    protected double getRPM() {
        return leftMotor.getVelocity().getValueAsDouble() * 60;
    }

    @Override
    protected void setVoltage(double voltage) {
        leftMotor.setVoltage(voltage);
    }

    @Override
    protected double getVoltage() {
        return leftMotor.getSupplyVoltage().getValueAsDouble();
    }
}
