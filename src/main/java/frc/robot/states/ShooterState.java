// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.states;

import frc.robot.util.Hub.Targets;

/** The state that the Shooter is in. */
public enum ShooterState {
    IDLE(0),
    DISABLED(0),
    CHARGING(3500),
    SHOOTING(3500);

    private final double rpm;
    ShooterState(double rpm) {
        this.rpm = rpm;
    }
    

    public double getRPM() {
        return rpm;
    }

    public double getZoneRPM(Targets zone) {
        if (zone == null) {
            return 0;
        }
        switch (zone) {
            case ZONE1:
                return 1;
            case ZONE2:
                return 2;
            case ZONE3:
                return 3;
            case ZONE4:
                return 4;
            default:
            return 0;
        }
    }
}