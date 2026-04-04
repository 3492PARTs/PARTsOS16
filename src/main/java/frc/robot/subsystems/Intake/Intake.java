package frc.robot.subsystems.Intake;

import org.parts3492.partslib.PARTsUnit;
import org.parts3492.partslib.PARTsUnit.PARTsUnitType;
import org.parts3492.partslib.command.PARTsCommandUtils;
import org.parts3492.partslib.command.PARTsSubsystem;

import frc.robot.RobotContainer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.HopperConstants;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.IntakeConstants.IntakeState;
import frc.robot.constants.RobotConstants;

public abstract class Intake extends PARTsSubsystem {

    IntakeState intakeState = IntakeState.IDLE;

    protected boolean debug = false;
    private Command toggleDebug = Commands.runOnce(() -> {
        debug = !debug;
        partsNT.putDouble("Intake Speed", 0, !RobotConstants.COMPETITION);
        partsNT.putDouble("Pivot Speed", 0, !RobotConstants.COMPETITION);
    }).ignoringDisable(true);

    ProfiledPIDController pivotPIDController;

    PIDController intakePIDController;
    SimpleMotorFeedforward intakeFeedforward;    

    public Intake() {
        super("Intake");
        if (RobotConstants.COMPETITION)
            debug = false;

        if (RobotContainer.debug || debug) {
            partsNT.putDouble("Intake Speed", 0, !RobotConstants.COMPETITION);
            partsNT.putDouble("Pivot Speed", 0, !RobotConstants.COMPETITION);
        }

        pivotPIDController = new ProfiledPIDController(IntakeConstants.PIVOT_P, IntakeConstants.PIVOT_I, IntakeConstants.PIVOT_D,
                new TrapezoidProfile.Constraints(IntakeConstants.INTAKE_MAX_VELOCITY,
                        IntakeConstants.INTAKE_MAX_ACCELERATION));
        pivotPIDController.setTolerance(IntakeConstants.PIVOT_PID_THRESHOLD);


        intakePIDController = new PIDController(IntakeConstants.INTAKE_P, IntakeConstants.INTAKE_I, IntakeConstants.INTAKE_D);
        intakeFeedforward = new SimpleMotorFeedforward(IntakeConstants.IntakeS, IntakeConstants.IntakeV, IntakeConstants.IntakeA);

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
        partsNT.putDouble("Intake RPM", getIntakeRPM(), debug);
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
            setIntakeVoltage(calculateRPMVoltage(partsNT.getDouble("Intake Speed", true)));
            setPivotSpeed(partsNT.getDouble("Pivot Speed", true));
        } 
        
        else {
            switch (intakeState) {
                case IDLE:
                case DISABLED:
                    setIntakeSpeed(intakeState.getRPM());
                    setPivotSpeed(0);
                    break;
                case INTAKING:
                case REVERSE:
                case HOME:
                    setIntakeVoltage(calculateRPMVoltage(intakeState.getRPM()));

                    pivotPIDController.setGoal(intakeState.getAngle().getValue());
                    double pidCalc = pivotPIDController.calculate(getPivotRotations().to(PARTsUnitType.Angle),
                            intakeState.getAngle().getValue());

                    partsNT.putBoolean("At goal", pivotPIDController.atSetpoint(), !RobotConstants.COMPETITION);
                    partsNT.putDouble("State Angle", intakeState.getAngle().getValue(), !RobotConstants.COMPETITION);

                    setPivotVoltage(pivotPIDController.atGoal() ? 0: pidCalc);
                    break;
                case MANUALPIVOT:
                    break;
                case SHOOTING:
                    setIntakeVoltage(calculateRPMVoltage(intakeState.getRPM()));

                    double getGoal = pivotPIDController.getGoal().position;
                    if (getGoal == 40 && pivotPIDController.atGoal()) {
                        getGoal = 90;
                    } else if (getGoal == 90 && pivotPIDController.atGoal()) {
                        getGoal = 40;
                    } else if (getGoal != 90 && getGoal != 40) {
                        getGoal = 40;
                    }
                    pivotPIDController.setGoal(getGoal);
                    pidCalc = pivotPIDController.calculate(getPivotRotations().to(PARTsUnitType.Angle),
                            getGoal);

                    partsNT.putBoolean("At goal", pivotPIDController.atSetpoint(), !RobotConstants.COMPETITION);
                    partsNT.putDouble("State Angle", intakeState.getAngle().getValue(), !RobotConstants.COMPETITION);
                    partsNT.putDouble("Pivot Goal", getGoal, !RobotConstants.COMPETITION);

                    setPivotVoltage(pidCalc);
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
        partsLogger.logDouble("Pivot Position", getPivotRotations().to(PARTsUnitType.Angle),
                RobotContainer.debug || debug);
        partsLogger.logDouble("Intake Speed", getIntakeSpeed(), RobotContainer.debug || debug);
        partsLogger.logString("Intake State", intakeState.toString(), RobotContainer.debug || debug);
    }
    // endregion

    // region Custom Public Functions
    public abstract void setIntakeSpeed(double speed);

    public abstract void setPivotSpeed(double speed);

    public abstract double getIntakeSpeed();

    public abstract double getIntakeRPM();

    public abstract PARTsUnit getPivotRotations();

    public abstract void setPivotVoltage(double voltage);

    public abstract void setIntakeVoltage(double voltage);

    public abstract double getPivotRotationSpeed();

    public abstract Command oneNinetyArm();

    public abstract Command zeroArm();

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
        return PARTsCommandUtils.setCommandName("Intake.idle", Commands.runOnce(() -> {
            intakeState = IntakeState.IDLE;
        }));
    }

    public Command hold() {
        return PARTsCommandUtils.setCommandName("Intake.hold", Commands.runOnce(() -> {
            IntakeState.HOLD.setAngle(new PARTsUnit(getPivotRotations().toPARTsUnit(PARTsUnitType.Angle).getValue(),
                    PARTsUnitType.Angle));
            intakeState = IntakeState.HOLD;
        }));
    }

    public Command home() {
        return PARTsCommandUtils.setCommandName("Intake.home", Commands.runOnce(() -> {
            intakeState = IntakeState.HOME;
        }));
    }

    public Command reverse() {
        return PARTsCommandUtils.setCommandName("Intake.reverse", Commands.runOnce(() -> {
            intakeState = IntakeState.REVERSE;
        }));
    }

    public Command manualPivot(double speed) {
        return PARTsCommandUtils.setCommandName("Intake.manualPivot", this.runOnce(() -> {
            intakeState = IntakeState.MANUALPIVOT;
            setPivotSpeed(speed);
        }));
    }

    // Intake roller same as hopper roller.
    private double calculateRPMVoltage(double rpm) {
        intakePIDController.setSetpoint(rpm);
        double pidCalc = intakePIDController.calculate(getIntakeRPM(), rpm);
        double ffCalc = intakeFeedforward.calculate((intakePIDController.getSetpoint() * Math.PI
                * HopperConstants.HOPPER_ROLLER_RADIUS.to(PARTsUnitType.Meter) * 2) / 60);

        double voltage = pidCalc + ffCalc;

        return voltage;
    }
    // endregion
}
