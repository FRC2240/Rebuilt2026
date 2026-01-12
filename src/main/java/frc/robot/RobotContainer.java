// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.subsystems.drivetrain.DriveCommands;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.utils.Field;

public class RobotContainer {
    private final CommandXboxController joystick = new CommandXboxController(0);

    public static final Drivetrain drivetrain = new Drivetrain();
    public final DriveCommands driveCommands = new DriveCommands(drivetrain, joystick);

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
                drivetrain.applyRequest(() -> idle).ignoringDisable(true));

        drivetrain.setDefaultCommand(driveCommands.driveWithJoystick());
        Field.inAllianceZone.whileTrue(driveCommands.drive(driveCommands.driveWithController(),driveCommands.rotateToFacePoint(()-> Field.HUB_CENTER_TRANSLATION.get())));

        // Reset the field-centric heading on left bumper press.
        joystick.leftBumper().whileTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
    }

    public Command getAutonomousCommand() {
        return Commands.none();
    }
}
