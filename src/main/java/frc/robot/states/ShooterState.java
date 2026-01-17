// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.states;

import frc.robot.constants.ShooterConstants;

/** The state that the shooter is in. */
public enum ShooterState {
    IDLE(0),
    DISABLED(0),
    CHARGING(0),
    SHOOTING(ShooterConstants.SHOOTER_SPEED);

    private final double speed;
    ShooterState(double speed) {
        this.speed = speed;
    }
    

    public double getSpeed() {
        return speed;
    }
}