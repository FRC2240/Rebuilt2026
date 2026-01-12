// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.subsystems.drivetrain.DriveCommands;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class RobotContainer {
    private final SendableChooser<Command> autoChooser;
    private final CommandXboxController joystick = new CommandXboxController(0);

    public static final Drivetrain drivetrain = new Drivetrain();
    public final DriveCommands driveCommands = new DriveCommands(drivetrain, joystick);

    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        configureBindings();
    }

    private void configureBindings() {
        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
                drivetrain.applyRequest(() -> idle).ignoringDisable(true));

        drivetrain.setDefaultCommand(driveCommands.driveWithJoystick());
        // Field.inAllianceZone.whileTrue(driveCommands.drive(driveCommands.driveWithController(),driveCommands.rotateToFacePoint(()->
        // Field.HUB_CENTER_TRANSLATION.get())));

        // Reset the field-centric heading on left bumper press.
        joystick.leftBumper().whileTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
