// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashSet;
import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.CandleConstants;
import frc.robot.states.CandleState;
import org.parts3492.partslib.command.PARTsCommandUtils;
import org.parts3492.partslib.PARTsCandle;

/** Add your docs here. */
public class Candle extends PARTsCandle {
    private CandleState candleState = CandleState.DISABLED;
    private Set<CandleState> candleStates = new HashSet<>();

    public Candle() {
        super("Candle", CandleConstants.CAN_ID, CandleConstants.LED_LENGTH, CandleConstants.CAN_BUS_NAME);
    }

    /*---------------------------------- Custom Public Functions ----------------------------------*/
    public void addState(CandleState state) {
        candleStates.add(state);

        setState();
    }

    public void removeState(CandleState state) {
        candleStates.remove(state);

        setState();
    }

    public CandleState getState() {
        return candleState;
    }

    public Command commandAddState(CandleState state) {
        return PARTsCommandUtils.setCommandName("Candle.commandAddState",
                Commands.runOnce(() -> addState(state)).ignoringDisable(true));
    }

    public Command commandRemoveState(CandleState state) {
        return PARTsCommandUtils.setCommandName("Candle.commandRemoveState",
            Commands.runOnce(() -> removeState(state)).ignoringDisable(true));
    }

    /*---------------------------------- Custom Private Functions ---------------------------------*/
    private void setState() {

        // This picks the order of states to display
        /*if (candleStates.contains(CandleState.ELEVATOR_ERROR))
            candleState = CandleState.ELEVATOR_ERROR;
        else if (candleStates.contains(CandleState.CORAL_LASER_EXIT_ERROR))
            candleState = CandleState.CORAL_LASER_EXIT_ERROR;
        else if (candleStates.contains(CandleState.CORAL_LASER_ENTRY_ERROR))
            candleState = CandleState.CORAL_LASER_ENTRY_ERROR;
        else if (candleStates.contains(CandleState.CORAL_ENTERING))
            candleState = CandleState.CORAL_ENTERING;
        else if (candleStates.contains(CandleState.AUTO_ALIGN))
            candleState = CandleState.AUTO_ALIGN;
        else if (candleStates.contains(CandleState.SCORING))
            candleState = CandleState.SCORING;
        else if (candleStates.contains(CandleState.HAS_CORAL))
            candleState = CandleState.HAS_CORAL;
        else if (candleStates.contains(CandleState.FINE_GRAIN_DRIVE))
            candleState = CandleState.FINE_GRAIN_DRIVE;*/
        if (candleStates.contains(CandleState.ACTIVE_SHOOTING))
            candleState = CandleState.ACTIVE_SHOOTING;
        else if (candleStates.contains(CandleState.SHOOTING))
            candleState = CandleState.SHOOTING;
        else if (candleStates.contains(CandleState.IDLE))
            candleState = CandleState.IDLE;
        else if (candleStates.contains(CandleState.DISABLED))
            candleState = CandleState.DISABLED; 

        setStateAnimation();
    }

    private void setStateAnimation() {
        // Maps state to animation
        switch (candleState) {
            /*case ELEVATOR_ERROR:
                runLarsonAnimation(Color.ORANGE, 0.75, BounceMode.Center, 7);
                break;
            case CORAL_LASER_EXIT_ERROR:
                runLarsonAnimation(Color.RED, 0.75, BounceMode.Center, 7);
                break;
            case CORAL_LASER_ENTRY_ERROR:
                runLarsonAnimation(Color.YELLOW, 0.75, BounceMode.Center, 7);
                break;
            case FINE_GRAIN_DRIVE:
                runTwinkleAnimation(Color.ORANGE, .75, TwinklePercent.Percent30, 0);
                break;
            case CORAL_ENTERING:
                runFadeAnimation(Color.PURPLE, .75);
                break;
            case HAS_CORAL:
                runFadeAnimation(Color.GREEN, .75);
                break;
            case SCORING:
                runRainbowAnimation();
                break;*/
            case SHOOTING:
                break;
            case ACTIVE_SHOOTING:
                break;
            case IDLE:
                runFadeAnimation(Color.BLUE, .75);
                break;
            case DISABLED:
                setColor(Color.BLUE);
                break;
            default:
                break;
        }
    }

    /*-------------------------------- Generic Subsystem Functions --------------------------------*/

    @Override
    public void outputTelemetry() {
        super.outputTelemetry();
        super.partsNT.putString("State", candleState.toString());
    }
}
