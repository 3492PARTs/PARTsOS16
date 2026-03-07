package frc.robot.subsystems.Intake;

import static edu.wpi.first.units.Units.Radian;

import org.parts3492.partslib.PARTsUnit;
import org.parts3492.partslib.PARTsUnit.PARTsUnitType;
import org.parts3492.partslib.command.PARTsCommandUtils;
import org.parts3492.partslib.command.PARTsSubsystem;
import org.parts3492.partslib.input.PARTsButtonBoxController;

import frc.robot.RobotContainer;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.IntakeConstants.IntakeState;
import frc.robot.constants.RobotConstants;

public abstract class Intake extends PARTsSubsystem {

    IntakeState intakeState = IntakeState.IDLE;

    protected boolean debug = false;
    private Command toggleDebug = Commands.runOnce(()-> debug = !debug).ignoringDisable(true);

    ProfiledPIDController intakePIDController;
    ArmFeedforward intakeFeedForward;

    public Intake() {
        super("Intake");
        if (RobotConstants.COMPETITION) debug = false;

        if (RobotContainer.debug || debug) {
            partsNT.putDouble("Intake Speed", 0, true);
            partsNT.putDouble("Pivot Speed", 0, true);
        }

        intakePIDController = new ProfiledPIDController(IntakeConstants.P, IntakeConstants.I, IntakeConstants.D,
                new TrapezoidProfile.Constraints(IntakeConstants.INTAKE_MAX_VELOCITY,
                        IntakeConstants.INTAKE_MAX_ACCELERATION));
        intakeFeedForward = new ArmFeedforward(IntakeConstants.S, IntakeConstants.V, IntakeConstants.A);
        intakePIDController.setTolerance(IntakeConstants.PID_THRESHOLD);

        partsNT.putSmartDashboardSendable("Toggle Intake Debug", toggleDebug, !RobotConstants.COMPETITION);
    }

    public IntakeState getState() {
        return intakeState;
    }

    // region Generic Subsystem Functions
    @Override
    public void outputTelemetry() {
        partsNT.putDouble("Pivot Angle", getPivotRotations().to(PARTsUnitType.Angle), true);
        partsNT.putDouble("Current Intake Speed", getIntakeSpeed(), RobotContainer.debug || debug);
        partsNT.putString("Intake State", intakeState.toString(), !RobotConstants.COMPETITION);
        partsNT.putBoolean("Intake Debug Active", debug, !RobotConstants.COMPETITION);
    }

    @Override
    public void stop() {
        intakeState = IntakeState.DISABLED;
    }

    @Override
    public void reset() {
        intakeState = IntakeState.IDLE;
    }

    @Override
    public void periodic() {
        if (RobotContainer.debug || debug) {
            setIntakeSpeed(partsNT.getDouble("Intake Speed", true));
            setPivotSpeed(partsNT.getDouble("Pivot Speed", true));
        } else {
            switch (intakeState) {
                case DISABLED:
                    setIntakeSpeed(intakeState.getSpeed());
                    setPivotSpeed(0);
                    break;
                case IDLE:
                case INTAKING:
                case OUTTAKING:
                case SHOOTING:
                case HOME:
                case TRAVELING:
                    setIntakeSpeed(intakeState.getSpeed());

                    intakePIDController.setGoal(intakeState.getAngle().getValue());
                    double pidCalc = intakePIDController.calculate(getPivotRotations().to(PARTsUnitType.Angle),
                            intakeState.getAngle().getValue());

                    partsNT.putBoolean("At goal", intakePIDController.atSetpoint(), true);
                    partsNT.putDouble("State Angle", intakeState.getAngle().getValue(), true);

                    setPivotVoltage(pidCalc);
                    break;
                case MANUALPIVOT:
                    break;
                default:
                    setIntakeSpeed(0);
                    setPivotSpeed(0);
                    break;
            }
        }
    }

    @Override
    public void log() {
        partsLogger.logDouble("Pivot Position", getPivotRotations().to(PARTsUnitType.Angle), RobotContainer.debug || debug);
        partsLogger.logDouble("Intake Speed", getIntakeSpeed(), RobotContainer.debug || debug);
        partsLogger.logString("Intake State", intakeState.toString(), RobotContainer.debug || debug);
    }
    // endregion

    // region Custom Public Functions
    public abstract void setIntakeSpeed(double speed);

    public abstract void setPivotSpeed(double speed);

    public abstract double getIntakeSpeed();

    public abstract PARTsUnit getPivotRotations();

    public abstract void setPivotVoltage(double voltage);

    public abstract double getPivotRotationSpeed();

    public Command intake() {
        return PARTsCommandUtils.setCommandName("Intake.intake", Commands.runOnce(() -> {
            intakeState = IntakeState.INTAKING;
        }));
    }

    public Command intakeShooting() {
        return PARTsCommandUtils.setCommandName("Intake.intakeShooting", Commands.runOnce(() -> {
            intakeState = IntakeState.SHOOTING;
        }));
    }

    public Command idle() {
        return PARTsCommandUtils.setCommandName("Intake.intakeIdle", Commands.runOnce(() -> {
            IntakeState.IDLE.setAngle(new PARTsUnit(getPivotRotations().toPARTsUnit(PARTsUnitType.Angle).getValue(), PARTsUnitType.Angle));
            intakeState = IntakeState.IDLE;
        }));
    }

    public Command home() {
        return PARTsCommandUtils.setCommandName("Intake.home", Commands.runOnce(() -> {
            intakeState = IntakeState.HOME;
        }));
    }

    public Command travel() {
        return PARTsCommandUtils.setCommandName("Intake.travel", Commands.runOnce(() -> {
            intakeState = IntakeState.TRAVELING;
        }));
    }

    public Command manualPivot(double speed) {
        return PARTsCommandUtils.setCommandName("Intake.manualPivot", this.runOnce(() -> {
            intakeState = IntakeState.MANUALPIVOT;
            setPivotSpeed(speed);
        }));
    }
    // endregion
}
