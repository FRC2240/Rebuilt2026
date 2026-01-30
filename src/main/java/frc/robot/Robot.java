// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.utils.Field;
import frc.robot.utils.logging.BonusMethods;
import frc.robot.utils.logging.Register;
import frc.robot.utils.logging.VisualLogger;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;

    public Robot() {
        DataLogManager.start();
        m_robotContainer = new RobotContainer();
    }

    @Override
    public void robotInit() {
        Register.registerT2d("VisualLogger/PassingTargets/Right");
        Register.registerT2d("VisualLogger/PassingTargets/Left");
        Register.registerT2dArray("VisualLogger/Lines/Line_Right");
        Register.registerT2dArray("VisualLogger/Lines/Line_Left");
        Register.registerT2dArray("VisualLogger/Lines/Active");

        VisualLogger.publishTranslation2d("VisualLogger/PassingTargets/Right", Field.PASSING_TARGET_RIGHT_TRANSLATION.get());
        VisualLogger.publishTranslation2d("VisualLogger/PassingTargets/Left", Field.PASSING_TARGET_LEFT_TRANSLATION.get());
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

        VisualLogger.publishTranslation2dArray("VisualLogger/Lines/Active",
            RobotContainer.drivetrain.getTranslation(),
            BonusMethods.BM_getTargetTranslation());
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
