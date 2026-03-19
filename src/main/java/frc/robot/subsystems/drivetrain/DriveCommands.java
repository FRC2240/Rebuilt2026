package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class DriveCommands extends SubsystemBase {
    private Drivetrain drivetrain;
    private CommandXboxController joystick;

    private final SwerveRequest.ApplyFieldSpeeds driveChassisSpeeds = new SwerveRequest.ApplyFieldSpeeds();
    SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveLimiter limiter;

    private boolean slowModeEnabled = false;

    public static class TranslationalVelocity {
        public LinearVelocity x;
        public LinearVelocity y;

        public TranslationalVelocity(LinearVelocity x, LinearVelocity y) {
            this.x = x;
            this.y = y;
        }

        public static TranslationalVelocity none() {
            return new TranslationalVelocity(MetersPerSecond.of(0), MetersPerSecond.of(0));
        }
    }

    // Constructor
    public DriveCommands(Drivetrain drivetrain, CommandXboxController controller) {
        this.drivetrain = drivetrain;
        this.joystick = controller;
        this.limiter = new SwerveLimiter(drivetrain);
    }

    /**
     * Gets the maximum drive speed, accounting for if the robot is in slow mode.
     * 
     * @return The current maximum speed
     */
    private LinearVelocity getMaxDriveSpeed() {
        return slowModeEnabled ? DriveConstants.MAX_SLOW_SPEED : DriveConstants.MAX_SPEED;
    }

    /**
     * Gets the maximum rotational speed, accounting for if the robot is in slow
     * mode.
     * 
     * @return The current maximum speed
     */
    private AngularVelocity getMaxRotationSpeed() {
        return slowModeEnabled ? DriveConstants.MAX_SLOW_ANGULAR_RATE : DriveConstants.MAX_ANGULAR_RATE;
    }

    /**
     * Command to toggle slow mode
     */
    public Command toggleSlowModeCommand() {
        return Commands.runOnce(() -> slowModeEnabled = !slowModeEnabled);
    }

    private double applyDeadband(double value, double deadband) {
        if (Math.abs(value) > deadband)
            return value;
        return 0;
    }

    /**
     * Puts the value to the power of exponent. Preserves the sign of the input
     */
    private double delinearize(double value, double exponent) {
        return Math.pow(Math.abs(value), exponent) * (value < 0 ? -1 : 1);
    }

    public Command brake() {
        return drivetrain.applyRequest(() -> brake);
    }

    /**
     * Drives the robot with the specified translation and rotation suppliers.
     * Note that the parameters can be null for no control.
     */
    public Command drive(Supplier<TranslationalVelocity> translationSupplier,
            Supplier<AngularVelocity> rotationSupplier) {
        TranslationalVelocity noTranslation = TranslationalVelocity.none();
        AngularVelocity noRotation = RadiansPerSecond.of(0);

        return drivetrain.applyRequest(() -> {
            TranslationalVelocity translation = translationSupplier != null ? translationSupplier.get() : noTranslation;
            AngularVelocity rotation = rotationSupplier != null ? rotationSupplier.get() : noRotation;

            ChassisSpeeds requestedSpeeds = new ChassisSpeeds(translation.x, translation.y, rotation);

            return driveChassisSpeeds.withSpeeds(limiter.calculate(requestedSpeeds));
        });
    }

    /**
     * Drives the robot with the specified translation and rotation suppliers.
     * Note that the parameters can be null for no control or no limit.
     */
    public Command drive(Supplier<TranslationalVelocity> translationSupplier,
            Supplier<AngularVelocity> rotationSupplier, LinearVelocity maxLinearVelocity,
            AngularVelocity maxAngularVelocity) {
        TranslationalVelocity noTranslation = TranslationalVelocity.none();
        AngularVelocity noRotation = RadiansPerSecond.of(0);

        return drive(() -> {
            if (translationSupplier == null)
                return noTranslation;
            if (maxLinearVelocity == null)
                return translationSupplier.get();

            TranslationalVelocity translation = translationSupplier.get();
            double requestedVelocity = Math.sqrt(
                    Math.pow(translation.x.in(MetersPerSecond), 2) + Math.pow(translation.y.in(MetersPerSecond), 2));
            if (requestedVelocity > maxLinearVelocity.in(MetersPerSecond)) {
                double scaleFactor =  maxLinearVelocity.in(MetersPerSecond) / requestedVelocity;
                translation.x = translation.x.times(scaleFactor);
                translation.y = translation.y.times(scaleFactor);
            }

            return translation;
        }, () -> {
            if (rotationSupplier == null)
                return noRotation;
            if (maxAngularVelocity == null)
                return rotationSupplier.get();

            double rps = rotationSupplier.get().in(RotationsPerSecond);
            return RotationsPerSecond.of(
                    Math.min(Math.max(rps, -maxAngularVelocity.in(RotationsPerSecond)),
                            maxAngularVelocity.in(RotationsPerSecond)));
        });
    }

    /**
     * Wraps a Command to stop it on joystick input after a timeout
     */
    public Command withJoystickOverride(Command cancelableCommand) {
        Timer timer = new Timer();
        return Commands.runOnce(() -> timer.start(), drivetrain).andThen(cancelableCommand.until(() -> {
            if (!timer.hasElapsed(DriveConstants.CONTROLLER_OVERRIDE_TIMEOUT))
                return false;
            return Math.abs(joystick.getLeftX()) > DriveConstants.CONTROLLER_OVERRIDE_THRESHOLD ||
                    Math.abs(joystick.getLeftY()) > DriveConstants.CONTROLLER_OVERRIDE_THRESHOLD ||
                    Math.abs(joystick.getRightX()) > DriveConstants.CONTROLLER_OVERRIDE_THRESHOLD;
        }));
    }

    /**
     * Drives the robot with the drive and rotation joysticks
     */
    public Command controlWithJoysticks() {
        return drive(driveWithJoystick(), rotateWithJoystick());
    };

    /**
     * Drives the robot to the specified position
     * Does not end when the target has been reached because the value in the pose
     * supplier can always change.
     * Note that joystick input stops the command
     */
    public Command driveToPose(Supplier<Pose2d> poseSupplier) {
        return withJoystickOverride(drive(
                driveToPoint(() -> poseSupplier.get().getTranslation()),
                rotateToRotation(() -> poseSupplier.get().getRotation())));
    }

    /**
     * Alternative drive to pose function with a single pose2d target.
     * The command ends after the target has been reached.
     * Note that joystick input stops the command
     */
    public Command driveToPose(Pose2d target) {
        return driveToPose(() -> target)
                .until(() -> {
                    // Stops the command once it's pose is within tolerance
                    Pose2d currentPose = drivetrain.getPose();
                    boolean translationFinished = target.getTranslation().minus(currentPose.getTranslation())
                            .getNorm() < DriveConstants.TRANSLATION_FINISHED_THRESHOLD.in(Meters);
                    boolean rotationFinished = Math.abs(target.getRotation().minus(currentPose.getRotation())
                            .getRadians()) < DriveConstants.ROTATION_FINISHED_THRESHOLD.in(Radians);
                    return translationFinished && rotationFinished;
                });
    }

public Command snapToRotationPosition(double targetAngleDegrees) { 
    return new InstantCommand(() -> { 
    rotateToRotation(() -> Rotation2d.fromDegrees(targetAngleDegrees)); 
    });
 }

 public Supplier<AngularVelocity> trenchAlign() {
    return rotateToRotation(() -> {
    double currentRotation = Math.abs(drivetrain.getHeading().getDegrees());

    if (currentRotation >= 90) {
        return Rotation2d.fromDegrees(180);
    }
    return Rotation2d.fromDegrees(0);
 });
 }

    /**
     * Transforms joystick input into translational velocity
     */
    public Supplier<TranslationalVelocity> driveWithJoystick() {
        TranslationalVelocity velocity = TranslationalVelocity.none();
        return () -> {
            // Inverts controls if on the blue alliance
            int allianceMultiplier = (DriverStation.getAlliance().isEmpty()
                    || DriverStation.getAlliance().get() == Alliance.Red) ? 1 : -1;

            velocity.x = getMaxDriveSpeed()
                    .times(delinearize(applyDeadband(joystick.getLeftY(), DriveConstants.CONTROLLER_DEADBAND), 1.5) * allianceMultiplier);
            velocity.y = getMaxDriveSpeed()
                    .times(delinearize(applyDeadband(joystick.getLeftX(), DriveConstants.CONTROLLER_DEADBAND), 1.5) * allianceMultiplier);
            return velocity;
        };
    }

    /**
     * Transforms joystick input into angular velocity
     */
    public Supplier<AngularVelocity> rotateWithJoystick() {
        return () -> {
            return getMaxRotationSpeed()
                    .times(applyDeadband(-joystick.getRightX(), DriveConstants.CONTROLLER_DEADBAND));
        };
    }

    /**
     * Calculates translational velocity to drive to the target translation supplied
     * by the supplier
     */
    public Supplier<TranslationalVelocity> driveToPoint(Supplier<Translation2d> targetTranslationSupplier) {
        TranslationalVelocity velocity = TranslationalVelocity.none();
        return () -> {
            Translation2d targetTranslation = targetTranslationSupplier.get();
            Translation2d currentTranslation = drivetrain.getTranslation();
            Translation2d translationToTarget = targetTranslation.minus(currentTranslation);

            Rotation2d directionOfTravel = translationToTarget.getAngle();
            double linearDistance = targetTranslation.minus(currentTranslation).getNorm();

            double velocityOutput = DriveConstants.TRANSLATION_PID_CONTROLLER.calculate(linearDistance, 0);
            // Limit the resulting velocity
            velocityOutput = Math.min(velocityOutput, getMaxDriveSpeed().in(MetersPerSecond));
            velocity.x = MetersPerSecond.of(-velocityOutput * directionOfTravel.getCos());
            velocity.y = MetersPerSecond.of(-velocityOutput * directionOfTravel.getSin());

            return velocity;
        };
    }

    /**
     * Drives the robot to the nearest point at a distance from a point
     */
    public Supplier<TranslationalVelocity> driveToDistanceFromPoint(Supplier<Translation2d> pointSupplier,
            Supplier<Distance> distanceSupplier) {
        return driveToPoint(() -> {
            Translation2d point = pointSupplier.get();
            Distance distance = distanceSupplier.get();

            Translation2d currentTranslation = drivetrain.getTranslation();
            Rotation2d angle = currentTranslation.minus(point).getAngle();

            return new Translation2d(distance.in(Meters), angle).plus(point);
        });
    }

    /**
     * Calculates angular velocity to rotate to the rotation specified by the
     * supplier
     */
    public Supplier<AngularVelocity> rotateToRotation(Supplier<Rotation2d> targetRotationSupplier) {
        return () -> {
            Rotation2d targetRotation = targetRotationSupplier.get();
            Rotation2d currentRotation = drivetrain.getHeading();
            double rotationDistance = currentRotation.minus(targetRotation).getRadians();

            double rotationalOutput = DriveConstants.ROTATION_PID_CONTROLLER.calculate(rotationDistance, 0);
            // Clamps the output to the max rotational speed
            rotationalOutput = Math.max(Math.min(rotationalOutput, getMaxRotationSpeed().in(RadiansPerSecond)),
                    -getMaxRotationSpeed().in(RadiansPerSecond));

            return RadiansPerSecond.of(rotationalOutput);
        };
    }

    /**
     * Calculates the angular velocity to rotate to face a point on the field.
     */
    public Supplier<AngularVelocity> rotateToFacePoint(Supplier<Translation2d> pointSupplier) {
        return rotateToRotation(() -> {
            Translation2d targetTranslation = pointSupplier.get();
            Translation2d currentTranslation = drivetrain.getTranslation();
            Translation2d translationToTarget = targetTranslation.minus(currentTranslation);
            Rotation2d rotationToPoint = translationToTarget.getAngle();
            return rotationToPoint;
        });
    }

    /*
     * public Supplier<AngularVelocity> rotateToAimAtHub(Supplier<LinearVelocity>
     * shooterExitGroundSpeedSupplier) {
     * return rotateToRotation(() -> {
     * Translation2d robotTranslation = drivetrain.getTranslation();
     * Translation2d hubTranslation = Field.HUB_CENTER_TRANSLATION.get();
     * 
     * ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
     * drivetrain.getState().Speeds,
     * drivetrain.getHeading());
     * 
     * Translation2d robotVelocities = new Translation2d(
     * fieldSpeeds.vxMetersPerSecond * DriveConstants.AIM_LATERAL_GAIN,
     * fieldSpeeds.vyMetersPerSecond * DriveConstants.AIM_LATERAL_GAIN);
     * 
     * Translation2d translationToHub = hubTranslation.minus(robotTranslation);
     * double exitSpeed = shooterExitGroundSpeedSupplier.get().in(MetersPerSecond);
     * 
     * double timeOfFlight = translationToHub.getNorm() / exitSpeed;
     * Translation2d virtualTarget = translationToHub;
     * 
     * for (int i = 0; i < 3; i++) {
     * virtualTarget = translationToHub.minus(robotVelocities.times(timeOfFlight));
     * 
     * // Re-calculate time of flight based on the new distance to the virtual
     * target
     * timeOfFlight = virtualTarget.getNorm() / exitSpeed;
     * }
     * 
     * return virtualTarget.getAngle();
     * });
     * }
     */
}