// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.LimelightVision.MegaTagMode;
import org.parts3492.partslib.network.PARTsDashboard;
import org.parts3492.partslib.network.PARTsDashboard.DashboardTab;

import com.pathplanner.lib.commands.FollowPathCommand;

import org.parts3492.partslib.PARTsLogger;
import org.parts3492.partslib.network.PARTsNT;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    PARTsNT partsNT;
    PARTsLogger partsLogger;

    private final RobotContainer m_robotContainer;

    public Robot() {
        // This is needed for lasercan, without it causes robot to lag on boot
        // CanBridge.runTCP();

        // Make elastic dashboard file available
        WebServer.start(5800, Filesystem.getDeployDirectory().getPath());

        m_robotContainer = new RobotContainer();
        partsNT = new PARTsNT(this);
        partsLogger = new PARTsLogger();
        m_robotContainer.constructDashboard();

        partsLogger.logCommandScheduler();
        partsLogger.logPathPlanner();

        CameraServer.startAutomaticCapture();

        // m_robotContainer.resetStartPose();
        m_robotContainer.setMegaTagMode(MegaTagMode.MEGATAG1);

        DriverStation.silenceJoystickConnectionWarning(!isReal());

        m_robotContainer.getAlliance();

        CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());

        // in debugging start limelights in viewing to stop overheating
        // when debugging since the robot is on to long
        if (RobotConstants.DEBUGGING)
            m_robotContainer.setLimelightViewingMode();
        else
            m_robotContainer.setLimelightMainMode();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

        partsNT.putDouble("Match Time", DriverStation.getMatchTime());
        m_robotContainer.outputTelemetry();
        m_robotContainer.log();

        m_robotContainer.getAlliance();
    }

    @Override
    public void disabledInit() {
        m_robotContainer.stop();
        m_robotContainer.setCandleDisabledState();
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void disabledExit() {
    }

    @Override
    public void autonomousInit() {
        if (!RobotConstants.DEBUGGING) {
            // PARTsDashboard.setTab(DashboardTab.AUTONOMOUS);
        }
        m_robotContainer.runOnEnabled();
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {
    }

    @Override
    public void teleopInit() {
        if (!RobotConstants.DEBUGGING) {
            // PARTsDashboard.setTab(DashboardTab.TELEOPERATED);
        }

        m_robotContainer.runOnEnabled();
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void teleopExit() {
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void testExit() {
    }

    @Override
    public void simulationPeriodic() {
    }
}
