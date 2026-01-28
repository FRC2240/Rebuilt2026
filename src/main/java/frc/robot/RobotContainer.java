// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drivetrain.DriveCommands;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.utils.*;
import frc.robot.subsystems.vision.*;
import frc.robot.subsystems.candle.Candle;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

public class RobotContainer {
    private SendableChooser<Command> autoChooser;
    private final CommandXboxController joystick = new CommandXboxController(0);

    public static final Drivetrain drivetrain = new Drivetrain();
    public final DriveCommands driveCommands = new DriveCommands(drivetrain, joystick);
    public final Vision vision = Vision.createVision(drivetrain);
    public final Climber climber = new Climber();
    public final Intake intake = new Intake();
    public final Shooter shooter = new Shooter();
    public final Candle candle = new Candle(() -> true, () -> shooter.canHit);

    public final FieldSimulation sim = new FieldSimulation();

    public RobotContainer() {
        configureAutoChooser();
        configureBindings();
    }

    private void configureAutoChooser() {
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private void configureBindings() {
        // Testing suff. Please do not remove
        //sim.setDefaultCommand(Commands.run(() -> sim.shootWithRobotVelocity(drivetrain, Rotation2d.fromDegrees(50), MetersPerSecond.of(10)), sim));
        //drivetrain.setDefaultCommand(driveCommands.drive(driveCommands.driveWithJoystick(), driveCommands.rotateToAimAtHub(shooter::getBallVelocity)));

        // Drive with joysticks
        drivetrain.setDefaultCommand(driveCommands.controlWithJoysticks());

        // climber coast by default
        climber.setDefaultCommand(climber.coastCommand());

        // intake enabled by default
        intake.setDefaultCommand(intake.enableIntakeCommand());

        // climber coast on disable
        RobotModeTriggers.disabled().onTrue(climber.coastCommand().ignoringDisable(true));

        // Extend Climber
        joystick.povUp().toggleOnTrue(climber.extendCommand());

        // Deploy Intake
        joystick.povDown().toggleOnTrue(intake.extendIntakeCommand());

        // disable Intake
        joystick.leftTrigger().toggleOnTrue(intake.disableIntakeCommand());

        // Toggle slow mode
        joystick.back().onTrue(driveCommands.toggleSlowModeCommand());

        // Zero the gyro
        joystick.start().onTrue(Commands.runOnce(drivetrain::seedFieldCentric));

        // Shoot
        joystick.rightTrigger().whileTrue(shooter.setOutputCommand(5).andThen(shooter.shootCommand(true)));
        joystick.rightTrigger().onFalse(shooter.resetCommand());
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
