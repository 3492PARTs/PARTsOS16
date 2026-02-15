// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.states;

import frc.robot.constants.ShooterConstants;

/** The state that the shooter is in. */
public enum ShooterState {
    IDLE(0),
    DISABLED(0),
    CHARGING(ShooterConstants.SHOOTER_RPM),
    SHOOTING(ShooterConstants.SHOOTER_RPM);

    private final double rpm;
    ShooterState(double rpm) {
        this.rpm = rpm;
    }
    

    public double getRPM() {
        return rpm;
    }
}