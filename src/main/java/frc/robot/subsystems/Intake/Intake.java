package frc.robot.subsystems.Intake;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.constants.HopperConstants;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.IntakeConstants.IntakeState;
import frc.robot.constants.RobotConstants;
import org.littletonrobotics.junction.Logger;
import org.parts3492.partslib.PARTsUnit;
import org.parts3492.partslib.PARTsUnit.PARTsUnitType;
import org.parts3492.partslib.command.PARTsCommandUtils;
import org.parts3492.partslib.command.PARTsSubsystem;

public class Intake extends PARTsSubsystem {

  IntakeState intakeState = IntakeState.IDLE;
  IntakeIO intakeIO;
  PivotIO pivotIO;
  private IntakeInputsAutoLogged inputs = new IntakeInputsAutoLogged();

  protected boolean debug = false;
  private Command toggleDebug =
      Commands.runOnce(
              () -> {
                debug = !debug;
                partsNT.putDouble("Intake Speed", 0, !RobotConstants.COMPETITION);
                partsNT.putDouble("Pivot Speed", 0, !RobotConstants.COMPETITION);
              })
          .ignoringDisable(true);

  ProfiledPIDController pivotPIDController;
  PIDController intakePIDController;
  SimpleMotorFeedforward intakeFeedforward;

  public Intake(IntakeIO intakeIO, PivotIO pivotIO) {
    super("Intake");

    this.intakeIO = intakeIO;
    this.pivotIO = pivotIO;

    if (RobotConstants.COMPETITION) debug = false;

    if (RobotContainer.debug || debug) {
      partsNT.putDouble("Intake Speed", 0, !RobotConstants.COMPETITION);
      partsNT.putDouble("Pivot Speed", 0, !RobotConstants.COMPETITION);
    }

    pivotPIDController =
        new ProfiledPIDController(
            IntakeConstants.PIVOT_P,
            IntakeConstants.PIVOT_I,
            IntakeConstants.PIVOT_D,
            new TrapezoidProfile.Constraints(
                IntakeConstants.INTAKE_MAX_VELOCITY, IntakeConstants.INTAKE_MAX_ACCELERATION));
    pivotPIDController.setTolerance(IntakeConstants.PIVOT_PID_THRESHOLD);

    intakePIDController =
        new PIDController(
            IntakeConstants.INTAKE_P, IntakeConstants.INTAKE_I, IntakeConstants.INTAKE_D);
    intakeFeedforward =
        new SimpleMotorFeedforward(
            IntakeConstants.IntakeS, IntakeConstants.IntakeV, IntakeConstants.IntakeA);

    partsNT.putSmartDashboardSendable(
        "Toggle Intake Debug", toggleDebug, !RobotConstants.COMPETITION);
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
    intakeIO.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
    if (RobotContainer.debug || debug) {
      setIntakeVoltage(calculateRPMVoltage(partsNT.getDouble("Intake Speed", true)));
      setPivotSpeed(partsNT.getDouble("Pivot Speed", true));
    } else {
      switch (intakeState) {
        case IDLE:
        case DISABLED:
          setIntakeSpeed(intakeState.getRPM());
          setPivotSpeed(0);
          break;
        case REVERSE:
          setIntakeSpeed(-1);
          break;
        case INTAKING:
          setIntakeSpeed(1);
          // setIntakeVoltage(calculateRPMVoltage(intakeState.getRPM()));

          pivotPIDController.setGoal(intakeState.getAngle().getValue());
          double pidCalc =
              pivotPIDController.calculate(
                  getPivotRotations().to(PARTsUnitType.Angle), intakeState.getAngle().getValue());

          partsNT.putBoolean(
              "At goal", pivotPIDController.atSetpoint(), !RobotConstants.COMPETITION);
          partsNT.putDouble(
              "State Angle", intakeState.getAngle().getValue(), !RobotConstants.COMPETITION);
          setPivotVoltage(pivotPIDController.atGoal() ? 0 : pidCalc);
          break;
        case HOME:
          setIntakeSpeed(0);
          pivotPIDController.setGoal(intakeState.getAngle().getValue());
          pidCalc =
              pivotPIDController.calculate(
                  getPivotRotations().to(PARTsUnitType.Angle), intakeState.getAngle().getValue());
          setPivotVoltage(pivotPIDController.atGoal() ? 0 : pidCalc);
          break;
        case MANUALPIVOT:
          break;
        case SHOOTING:
          setIntakeSpeed(.3);
          // setIntakeVoltage(calculateRPMVoltage(intakeState.getRPM()));

          double getGoal = pivotPIDController.getGoal().position;
          if (getGoal == intakeState.getAngle().getValue() && pivotPIDController.atGoal()) {
            getGoal = intakeState.getAngle().getValue() + 30;
          } else if (getGoal == intakeState.getAngle().getValue() + 30
              && pivotPIDController.atGoal()) {
            getGoal = intakeState.getAngle().getValue();
          } else if (getGoal != intakeState.getAngle().getValue()
              && getGoal != intakeState.getAngle().getValue() + 30) {
            getGoal = intakeState.getAngle().getValue();
          }
          pivotPIDController.setGoal(getGoal);
          pidCalc =
              pivotPIDController.calculate(getPivotRotations().to(PARTsUnitType.Angle), getGoal);

          partsNT.putBoolean(
              "At goal", pivotPIDController.atSetpoint(), !RobotConstants.COMPETITION);
          partsNT.putDouble(
              "State Angle", intakeState.getAngle().getValue(), !RobotConstants.COMPETITION);
          partsNT.putDouble(
              "Pivot Goal", intakeState.getAngle().getValue(), !RobotConstants.COMPETITION);

          setPivotVoltage(pidCalc);
          break;
        default:
          setIntakeSpeed(0);
          setPivotSpeed(0);
          break;
      }
    }
  }

  public void log() {
    partsLogger.logDouble(
        "Pivot Position",
        getPivotRotations().to(PARTsUnitType.Angle),
        RobotContainer.debug || debug);
    partsLogger.logDouble("Intake Speed", getIntakeSpeed(), RobotContainer.debug || debug);
    partsLogger.logString("Intake State", intakeState.toString(), RobotContainer.debug || debug);
  }

  // endregion

  // region Custom Public Functions
  public void setIntakeSpeed(double speed) {
    intakeIO.setIntakeSpeed(speed);
  }

  public void setPivotSpeed(double speed) {
    pivotIO.setPivotSpeed(speed);
  }

  public double getIntakeSpeed() {
    return intakeIO.getIntakeSpeed();
  }

  public double getIntakeRPM() {
    return intakeIO.getIntakeRPM();
  }

  public PARTsUnit getPivotRotations() {
    return pivotIO.getPivotRotations();
  }

  public void setPivotVoltage(double voltage) {
    pivotIO.setPivotVoltage(voltage);
  }

  public void setIntakeVoltage(double voltage) {
    intakeIO.setIntakeVoltage(voltage);
  }

  public double getPivotRotationSpeed() {
    return pivotIO.getPivotRotationSpeed();
  }

  public Command oneNinetyArm() {
    return pivotIO.oneNinetyArm();
  }

  public Command zeroArm() {
    return pivotIO.zeroArm();
  }

  public Command intake() {
    return PARTsCommandUtils.setCommandName(
        "Intake.intake",
        Commands.runOnce(
            () -> {
              intakeState = IntakeState.INTAKING;
            }));
  }

  public Command intakeShooting() {
    return PARTsCommandUtils.setCommandName(
        "Intake.intakeShooting",
        Commands.runOnce(
            () -> {
              intakeState = IntakeState.SHOOTING;
            }));
  }

  public Command idle() {
    return PARTsCommandUtils.setCommandName(
        "Intake.idle",
        Commands.runOnce(
            () -> {
              intakeState = IntakeState.IDLE;
            }));
  }

  public Command hold() {
    return PARTsCommandUtils.setCommandName(
        "Intake.hold",
        Commands.runOnce(
            () -> {
              IntakeState.HOLD.setAngle(
                  new PARTsUnit(
                      getPivotRotations().toPARTsUnit(PARTsUnitType.Angle).getValue(),
                      PARTsUnitType.Angle));
              intakeState = IntakeState.HOLD;
            }));
  }

  public Command home() {
    return PARTsCommandUtils.setCommandName(
        "Intake.home",
        Commands.runOnce(
            () -> {
              intakeState = IntakeState.HOME;
            }));
  }

  public Command reverse() {
    return PARTsCommandUtils.setCommandName(
        "Intake.reverse",
        Commands.runOnce(
            () -> {
              intakeState = IntakeState.REVERSE;
            }));
  }

  public Command manualPivot(double speed) {
    return PARTsCommandUtils.setCommandName(
        "Intake.manualPivot",
        this.runOnce(
            () -> {
              intakeState = IntakeState.MANUALPIVOT;
              setPivotSpeed(speed);
            }));
  }

  // Intake roller same as hopper roller.
  private double calculateRPMVoltage(double rpm) {
    intakePIDController.setSetpoint(rpm);
    double pidCalc = intakePIDController.calculate(getIntakeRPM(), rpm);
    double ffCalc =
        intakeFeedforward.calculate(
            (intakePIDController.getSetpoint()
                    * Math.PI
                    * HopperConstants.HOPPER_ROLLER_RADIUS.to(PARTsUnitType.Meter)
                    * 2)
                / 60);

    double voltage = pidCalc + ffCalc;

    return voltage;
  }
  // endregion
}
