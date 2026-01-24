// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.subsystems.drivetrain.DriveCommands;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.utils.*;
import frc.robot.subsystems.vision.*;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.intake.intake;

public class RobotContainer {
    private SendableChooser<Command> autoChooser;
    private final CommandXboxController joystick = new CommandXboxController(0);


    public static final Drivetrain drivetrain = new Drivetrain();
    public final DriveCommands driveCommands = new DriveCommands(drivetrain, joystick);
    public final Vision vision = Vision.createVision(drivetrain);
    public final Climber climber = new Climber();
    public final intake intake = new intake();
    public final Shooter shooter = new Shooter();


    public final FieldSimulation sim = new FieldSimulation();

    public RobotContainer() {
        configurePathPlanner();
        configureBindings();
    }

    private void configurePathPlanner() {
        NamedCommands.registerCommand("hub_align",
                driveCommands.drive(null, driveCommands.rotateToFacePoint(() -> Field.HUB_CENTER_TRANSLATION.get())));

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private void configureBindings() {
        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
                drivetrain.applyRequest(() -> idle).ignoringDisable(true));
                

        joystick.povUp().toggleOnTrue(Commands.runOnce(() -> climber.extendCommand()));
        joystick.povDown().toggleOnTrue(Commands.runOnce(() -> intake.extendIntakeCommand()));
        joystick.leftTrigger().toggleOnTrue(Commands.runOnce(() -> intake.enableIntakeCommand()));
        joystick.back().toggleOnTrue(Commands.runOnce(() -> driveCommands.toggleSlowModeCommand()));
        joystick.start().onTrue(Commands.runOnce(() -> drivetrain.seedFieldCentric()));
        joystick.rightTrigger().whileTrue(Commands.runOnce(() -> shooter.shoot()));

    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
