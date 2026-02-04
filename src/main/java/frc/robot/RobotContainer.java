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
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.utils.*;
import frc.robot.subsystems.vision.*;
import frc.robot.subsystems.candle.Candle;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

public class RobotContainer {
    private SendableChooser<Command> autoChooser;
    private static final CommandXboxController joystick = new CommandXboxController(0);

    // Drivetrain (and joystick) is static to make the pose getting methods global.
    // This will be fixed later with a singelton utility class.
    public static final Drivetrain drivetrain = new Drivetrain(joystick);
    public final Vision vision = Vision.createVision(drivetrain);
    public final Climber climber = new Climber();
    public final Intake intake = new Intake();
    public final Shooter shooter = new Shooter();
    public final Spindexer spindexer = new Spindexer();
    public final ShootingController shootingController = new ShootingController(drivetrain, shooter, spindexer);
    public final Candle candle = new Candle( shootingController::isShooting, shootingController::hubShootRequirementsMet);


    public final FieldSimulation sim = new FieldSimulation();

    public RobotContainer() {
        configureAutoChooser();
        configureBindings();
        configureDefaults();
        addNamedCommands();
        RobotModeTriggers.autonomous().or(RobotModeTriggers.teleop()).onTrue(intake.extendIntakeCommand());
    }

    private void addNamedCommands() {

        // Shoot
        NamedCommands.registerCommand("shoot", shootingController.shoot());
    }

    private void configureAutoChooser() {
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private void configureBindings() {
        // Testing stuff. Please do not remove
        // sim.setDefaultCommand(Commands.run(() -> sim.shootWithRobotVelocity(drivetrain, Rotation2d.fromDegrees(50), MetersPerSecond.of(10)), sim));
        // drivetrain.setDefaultCommand(driveCommands.drive(driveCommands.driveWithJoystick(), driveCommands.rotateToAimAtHub(shooter::getBallVelocity)));

        // Climber Coast on Disable
        RobotModeTriggers.disabled().onTrue(climber.coastCommand().ignoringDisable(true));

        // Extend Climber
        joystick.povUp().toggleOnTrue(climber.extendCommand());

        // Enable Intake
        joystick.leftTrigger().toggleOnTrue(intake.enableIntakeCommand());

        // Toggle Slow Mode
        joystick.back().onTrue(drivetrain.commands.toggleSlowModeCommand());

        // Zero the Gyro
        joystick.start().onTrue(drivetrain.rezeroGyro());

        // Shoot
        joystick.rightTrigger().whileTrue(shootingController.shoot());
    }

    private void configureDefaults() {

        // Spindexer Disable
        spindexer.setDefaultCommand(spindexer.disableCommand());

        // Climber Coast
        climber.setDefaultCommand(climber.coastCommand());

        // Intake Enabled
        intake.setDefaultCommand(intake.enableIntakeCommand());

        // Drive with Joysticks
        drivetrain.setDefaultCommand(drivetrain.commands.controlWithJoysticks());

    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
