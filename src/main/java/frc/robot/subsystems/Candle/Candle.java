package frc.robot.subsystems.Candle;

// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import frc.robot.constants.CandleConstants;
import frc.robot.constants.CandleConstants.CandleState;
import org.littletonrobotics.junction.Logger;
import org.parts3492.partslib.PARTsCandle;

/** Add your docs here. */
public class Candle extends PARTsCandle<CandleState> {

  public Candle() {
    super(
        "Candle", CandleConstants.CAN_ID, CandleConstants.LED_LENGTH, CandleConstants.CAN_BUS_NAME);
  }

  @Override
  protected void computeState() {
    if (getAllStates().contains(CandleState.ACTIVE_SHOOTING)) setState(CandleState.ACTIVE_SHOOTING);
    else if (getAllStates().contains(CandleState.SHOOTING)) setState(CandleState.SHOOTING);
    else if (getAllStates().contains(CandleState.IDLE)) setState(CandleState.IDLE);
    else if (getAllStates().contains(CandleState.DISABLED)) setState(CandleState.DISABLED);

    // Maps state to animation
    switch (getState()) {
      case SHOOTING:
        runStrobeAnimation(Color.ORANGE_RED);
        break;
      case ACTIVE_SHOOTING:
        runStrobeAnimation(Color.SKY_BLUE);
        break;
      case IDLE:
        runFadeAnimation(Color.BLUE, 100);
        break;
      case DISABLED:
        setColor(Color.BLUE);
        break;
      default:
        break;
    }
  }

  @Override
  public void periodic() {
    super.periodic();

    Logger.recordOutput(String.format("%s/%s", getName(), "state"), getState());

    String[] states = new String[getAllStates().size()];
    getAllStates().toArray(states);

    Logger.recordOutput(String.format("%s/%s", getName(), "states"), states);
  }
}
