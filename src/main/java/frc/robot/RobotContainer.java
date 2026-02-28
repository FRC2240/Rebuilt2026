// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.utils.*;
import frc.robot.subsystems.vision.*;
import frc.robot.subsystems.candle.Candle;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

public class RobotContainer {
    private SendableChooser<Command> autoChooser;

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final Drivetrain drivetrain = new Drivetrain(joystick);
    public final Vision vision = Vision.createVision(drivetrain);
    public final Intake intake = new Intake();
    public final Shooter shooter = new Shooter();
    public final Spindexer spindexer = new Spindexer();
    public final ShootingController shootingController = new ShootingController(drivetrain, shooter, spindexer, intake);
    public final Candle candle = new Candle(shootingController::isShooting, shootingController::hubShootRequirementsMet);

    // public final FieldSimulation sim = new FieldSimulation();

    public RobotContainer() {
        addNamedCommands();
        configureAutoChooser();
        configureDefaults();
        configureBindings();
    }

    private void addNamedCommands() {
        // The shoot command is called as proxy to ensure that the default commands on the subsystems are called
        NamedCommands.registerCommand("shoot", shootingController.shoot().asProxy());
        // NamedCommands.registerCommand("intake", intake.enableIntakeCommand());
        // NamedCommands.registerCommand("intake_deploy", intake.pivot.extendCommand());
    }

    private void configureAutoChooser() {
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private void configureBindings() {
        // Disable Intake
        joystick.leftTrigger().toggleOnTrue(intake.disableIntakeCommand());

        // Toggle Slow Mode
        joystick.back().onTrue(drivetrain.commands.toggleSlowModeCommand());

        // Zero the Gyro
        joystick.start().onTrue(drivetrain.rezeroGyro());

        // Shoot
        joystick.rightTrigger().whileTrue(shootingController.shoot());

        // Reverse intake
        joystick.povDown().whileTrue(intake.reverseIntakeCommand());

        //joystick.y().whileTrue(shootingController.feed());

        //joystick.a().onTrue(intake.pivot.extendCommand());
        //joystick.x().onTrue(intake.pivot.contractCommand());
        //joystick.b().onTrue(intake.pivot.rampCommand());

        joystick.rightBumper().onTrue(intake.pivot.rezeroCommand());
        //joystick.leftBumper().onTrue(intake.enableIntakeSlowCommand());
    }

    private void configureDefaults() {
        drivetrain.setDefaultCommand(drivetrain.commands.controlWithJoysticks());

        // Spindexer is disabled by default
        spindexer.setDefaultCommand(spindexer.disableCommand());

        // Intake is enabled by default
        intake.setDefaultCommand(intake.enableIntakeCommand());

        // Feeder is disabled by default
        shooter.feeder.setDefaultCommand(shooter.feeder.disableCommand());

        // Shooter coasts when not used (power saving)
        shooter.setDefaultCommand(shooter.coastCommand());
        
        // Pivot is extended by default
        intake.pivot.setDefaultCommand(intake.pivot.extendCommand());
    }

    public Command getAutonomousCommand() {
        if (autoChooser.getSelected() == null) return null;

        return autoChooser.getSelected();
    }
}
