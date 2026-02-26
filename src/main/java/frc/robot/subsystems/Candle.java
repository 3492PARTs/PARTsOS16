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

    public Command addStateCmd(CandleState state) {
        return PARTsCommandUtils.setCommandName("Candle.commandAddState",
                Commands.runOnce(() -> addState(state)).ignoringDisable(true));
    }

    public Command removeStateCmd(CandleState state) {
        return PARTsCommandUtils.setCommandName("Candle.commandRemoveState",
            Commands.runOnce(() -> removeState(state)).ignoringDisable(true));
    }

    /*---------------------------------- Custom Private Functions ---------------------------------*/
    private void setState() {

        // This picks the order of states to display
        if (candleStates.contains(CandleState.SHOOT_CMD_SHOOTING))
            candleState = CandleState.SHOOT_CMD_SHOOTING;
        else if (candleStates.contains(CandleState.SHOOT_CMD_ACTIVE))
            candleState = CandleState.SHOOT_CMD_ACTIVE;
        else if (candleStates.contains(CandleState.IDLE))
            candleState = CandleState.IDLE;
        else if (candleStates.contains(CandleState.DISABLED))
            candleState = CandleState.DISABLED; 

        setStateAnimation();
    }

    private void setStateAnimation() {
        // Maps state to animation
        switch (candleState) {
            case SHOOT_CMD_SHOOTING:
                runTwinkleAnimation(Color.ORANGE, .75);
                break;
            case SHOOT_CMD_ACTIVE:
                runTwinkleAnimation(Color.BLUE, .75);
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
