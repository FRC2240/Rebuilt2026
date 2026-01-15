package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.utils.Field;

public class DriveCommands {
    private Drivetrain drivetrain;
    private CommandXboxController joystick;

    private final SwerveRequest.ApplyFieldSpeeds driveChassisSpeeds = new SwerveRequest.ApplyFieldSpeeds();
    private final SwerveLimiter limiter;

    private boolean slowModeEnabled = false;

    public static class TranslationalVelocity {
        public LinearVelocity x;
        public LinearVelocity y;
        public boolean cancel;

        public TranslationalVelocity(LinearVelocity x, LinearVelocity y, boolean cancel) {
            this.x = x;
            this.y = y;
            this.cancel = cancel;
        }

        public static TranslationalVelocity none() {
            return new TranslationalVelocity(MetersPerSecond.of(0), MetersPerSecond.of(0), false);
        }
    }

    public static class RotationalVelocity {
        public AngularVelocity rotation;
        public boolean cancel;

        public RotationalVelocity(AngularVelocity rotation, boolean cancel) {
            this.rotation = rotation;
            this.cancel = cancel;
        }

        public static RotationalVelocity none() {
            return new RotationalVelocity(RadiansPerSecond.zero(), false);
        }
    }

    // Constructor
    public DriveCommands(Drivetrain drivetrain, CommandXboxController controller) {
        this.drivetrain = drivetrain;
        this.joystick = controller;
        this.limiter = new SwerveLimiter(drivetrain, DriveConstants.MAX_WHEEL_ACCELERATION,
                DriveConstants.MAX_WHEEL_DECELERATION);
    }

    // Utility for switching controller values based on alliance
    private int getAllianceMultiplier() {
        return (DriverStation.getAlliance().isEmpty()
                || DriverStation.getAlliance().get() == Alliance.Red) ? 1 : -1;
    }

    // Functions for slow mode
    private LinearVelocity getMaxDriveSpeed() {
        return slowModeEnabled ? DriveConstants.MAX_SLOW_SPEED : DriveConstants.MAX_SPEED;
    }

    private AngularVelocity getMaxRotationSpeed() {
        return slowModeEnabled ? DriveConstants.MAX_SLOW_ANGULAR_RATE : DriveConstants.MAX_ANGULAR_RATE;
    }

    public Command toggleSlowModeCommand() {
        return Commands.runOnce(() -> slowModeEnabled = !slowModeEnabled);
    }

    private double applyDeadband(double value, double deadband) {
        if (Math.abs(value) > deadband)
            return value;
        return 0;
    }

    // The main command that the DriveCommands class exposes. Allows users to
    // control translational and rotational velocities individually.
    public Command drive(Supplier<TranslationalVelocity> translationSupplier,
            Supplier<RotationalVelocity> rotationSupplier) {

        return drivetrain.applyRequest(() -> {
            TranslationalVelocity translation = translationSupplier.get();
            RotationalVelocity rotation = rotationSupplier.get();

            ChassisSpeeds requestedSpeeds = new ChassisSpeeds(translation.x, translation.y, rotation.rotation);

            return driveChassisSpeeds.withSpeeds(limiter.calculate(requestedSpeeds));
        }).until(() -> {
            // If either supplier tells the command to cancel, stops the command
            return translationSupplier.get().cancel == true ||
                    rotationSupplier.get().cancel == true;
        });
    }

    // Premade commands for easy/readable use
    public Command driveWithJoystick() {
        return drive(driveWithController(), rotateWithController());
    };

    public Command driveToPose(Supplier<Pose2d> poseSupplier) {
        return drive(driveToPoint(() -> poseSupplier.get().getTranslation()),
                rotateToRotation(() -> poseSupplier.get().getRotation()));
    }

    // Alternate drive to pose command with one target
    // and finishes when the pose is reached
    public Command driveToPose(Pose2d target) {
        Translation2d translation = target.getTranslation();
        Rotation2d rotation = target.getRotation();
        return drive(driveToPoint(() -> translation),
                rotateToRotation(() -> rotation))
                .until(() -> {
                    Pose2d currentPose = drivetrain.getPose();
                    boolean translationFinished = translation.minus(currentPose.getTranslation())
                            .getNorm() < DriveConstants.TRANSLATION_FINISHED_THRESHOLD.in(Meters);
                    boolean rotationFinished = Math.abs(rotation.minus(currentPose.getRotation())
                            .getRadians()) < DriveConstants.ROTATION_FINISHED_THRESHOLD.in(Radians);
                    return translationFinished && rotationFinished;
                });
    }

    // Suppliers for no control
    public Supplier<TranslationalVelocity> driveNone = () -> TranslationalVelocity.none();
    public Supplier<RotationalVelocity> rotateNone = () -> RotationalVelocity.none();

    // Suppliers for controlling with the joystick
    public Supplier<TranslationalVelocity> driveWithController() {
        TranslationalVelocity velocity = TranslationalVelocity.none();
        return () -> {
            int allianceMultiplier = getAllianceMultiplier();
            velocity.x = getMaxDriveSpeed()
                    .times(applyDeadband(joystick.getLeftY(), DriveConstants.CONTROLLER_DEADBAND) * allianceMultiplier);
            velocity.y = getMaxDriveSpeed()
                    .times(applyDeadband(joystick.getLeftX(), DriveConstants.CONTROLLER_DEADBAND) * allianceMultiplier);
            return velocity;
        };
    }

    public Supplier<RotationalVelocity> rotateWithController() {
        RotationalVelocity rotation = RotationalVelocity.none();
        return () -> {
            rotation.rotation = getMaxRotationSpeed()
                    .times(applyDeadband(-joystick.getRightX(), DriveConstants.CONTROLLER_DEADBAND));
            return rotation;
        };
    }

    // Suppliers for navigating to a certain point or rotation.
    public Supplier<TranslationalVelocity> driveToPoint(Supplier<Translation2d> targetTranslationSupplier) {
        TranslationalVelocity velocity = TranslationalVelocity.none();
        Timer timer = new Timer();
        timer.start();
        return () -> {
            Translation2d targetTranslation = targetTranslationSupplier.get();
            Translation2d currentTranslation = drivetrain.getTranslation();
            Translation2d translationToTarget = targetTranslation.minus(currentTranslation);

            Rotation2d directionOfTravel = translationToTarget.getAngle();
            double linearDistance = targetTranslation.minus(currentTranslation).getNorm();

            double velocityOutput = DriveConstants.TRANSLATION_PID_CONTROLLER.calculate(linearDistance, 0);
            velocityOutput = Math.min(velocityOutput, getMaxDriveSpeed().in(MetersPerSecond));

            velocity.x = MetersPerSecond.of(-velocityOutput * directionOfTravel.getCos());
            velocity.y = MetersPerSecond.of(-velocityOutput * directionOfTravel.getSin());

            velocity.cancel = (joystick.getLeftX() > DriveConstants.CONTROLLER_OVERRIDE_THRESHOLD ||
                    joystick.getLeftY() > DriveConstants.CONTROLLER_OVERRIDE_THRESHOLD) &&
                    timer.hasElapsed(DriveConstants.CONTROLLER_OVERRIDE_TIMEOUT);

            return velocity;
        };
    }

    public Supplier<RotationalVelocity> rotateToRotation(Supplier<Rotation2d> targetRotationSupplier) {
        RotationalVelocity rotation = RotationalVelocity.none();
        Timer timer = new Timer();
        timer.start();
        return () -> {
            Rotation2d targetRotation = targetRotationSupplier.get();
            Rotation2d currentRotation = drivetrain.getHeading();
            double rotationDistance = currentRotation.minus(targetRotation).getRadians();

            double rotationalOutput = DriveConstants.ROTATION_PID_CONTROLLER.calculate(rotationDistance, 0);
            rotationalOutput = Math.max(Math.min(rotationalOutput, getMaxRotationSpeed().in(RadiansPerSecond)),
                    -getMaxRotationSpeed().in(RadiansPerSecond));

            rotation.rotation = RadiansPerSecond.of(rotationalOutput);
            rotation.cancel = joystick.getRightX() > DriveConstants.CONTROLLER_OVERRIDE_THRESHOLD &&
                    timer.hasElapsed(DriveConstants.CONTROLLER_OVERRIDE_TIMEOUT);

            return rotation;
        };
    }

    // Locks the robot's heading to face a point on the field
    public Supplier<RotationalVelocity> rotateToFacePoint(Supplier<Translation2d> pointSupplier) {
        return rotateToRotation(() -> {
            Translation2d targetTranslation = pointSupplier.get();
            Translation2d currentTranslation = drivetrain.getTranslation();
            Translation2d translationToTarget = targetTranslation.minus(currentTranslation);
            Rotation2d rotationToPoint = translationToTarget.getAngle();
            return rotationToPoint;
        });
    }

    public Supplier<RotationalVelocity> aimAtHub() {
        return rotateToFacePoint(() -> Field.HUB_CENTER_TRANSLATION.get());
    }
}
