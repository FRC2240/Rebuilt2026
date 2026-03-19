// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Translation2d;
import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;

    public Robot() {
        DataLogManager.start();
        SignalLogger.enableAutoLogging(false);
        m_robotContainer = new RobotContainer();
    }

    @Override
    public void robotInit() {
        VisualLogger.publishPoint("PassingTargets/Right", Field.PASSING_TARGET_RIGHT_TRANSLATION.getBlueTranslation());
        VisualLogger.publishPoint("PassingTargets/Left", Field.PASSING_TARGET_LEFT_TRANSLATION.getBlueTranslation());

        VisualLogger.publishLine("Lines/Left", new Translation2d(Field.FIELD_LENGTH.div(2), Field.FIELD_WIDTH.div(2)), Field.PASSING_TARGET_LEFT_TRANSLATION.getBlueTranslation());
        VisualLogger.publishLine("Lines/Right", new Translation2d(Field.FIELD_LENGTH.div(2), Field.FIELD_WIDTH.div(2)), Field.PASSING_TARGET_RIGHT_TRANSLATION.getBlueTranslation());


        WebServer.start(5800, Filesystem.getDeployDirectory().getPath());
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        VisualLogger.publishPoint("Deadzone/Point", new Translation2d(LogDataMethods.isInDeadZone(), LogDataMethods.isInDeadZone()));

        VisualLogger.publishLine("Lines/Active",
            RobotContainer.drivetrain.getTranslation(),
            LogDataMethods.shooterGetTargetLogic());

        VisualLogger.publishPolygon("Testing/Triangle", //158.85 is midline
            new Translation2d[] { new Translation2d(Field.FIELD_LENGTH.div(2), Field.FIELD_WIDTH.div(2)), 
                new Translation2d(Inches.of(200), Field.FIELD_WIDTH.div(2).minus(Inches.of(50))), 
                new Translation2d(Inches.of(200), Field.FIELD_WIDTH.div(2).plus(Inches.of(50)))});
    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void disabledExit() {
    }

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
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
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
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