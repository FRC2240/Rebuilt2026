package frc.robot.utils;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drivetrain.DriveCommands;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.shooter.ShooterConstants;

public class ShootingController extends SubsystemBase {
    private static final AngularVelocity SHOOTER_VELOCITY_THRESHOLD = RotationsPerSecond.of(1);
    private static final Angle DRIVETRAIN_HEADING_THRESHOLD = Degrees.of(5);

    private static final Distance MIN_DISTANCE_FROM_HUB = Meters.of(1.5);
    private static final Distance MAX_DISTANCE_FROM_HUB = Meters.of(5);

    private final Drivetrain drivetrain;
    private final Shooter shooter;
    private final Spindexer spindexer;

    // Inches to Rotations Per Second
    private final InterpolatingDoubleTreeMap distanceToVelocityMap = new InterpolatingDoubleTreeMap();

    private boolean isCurrentlyShooting = false;

    public static StructArrayPublisher<Translation2d> targetLinePublisher = NetworkTableInstance.getDefault()
            .getStructArrayTopic(("Target_Line"), Translation2d.struct).publish();

    public ShootingController(Drivetrain drivetrain, Shooter shooter, Spindexer spindexer) {
        this.drivetrain = drivetrain;
        this.shooter = shooter;
        this.spindexer = spindexer;

        // Set values for the tree map
        distanceToVelocityMap.put(0., 0.);
        distanceToVelocityMap.put(10., 10.);
    }

    public boolean isShooting() {
        return isCurrentlyShooting;
    }

    private AngularVelocity getShooterVelocityForPosition() {
        // This is abstracted to it's own method to avoid problems if switching units in
        // the tree map
        return RotationsPerSecond.of(distanceToVelocityMap.get(Field.getDistanceToHub().in(Inches)));
    }

    // These boolean functions are in this class instead of their respective
    // subsystems. This is because this class holds the constants such as tolerances
    // for aiming and shooting so holding the methods here avoids a complex web of
    // imports to and from subsystems. TLDR: Reduces complexity

    private boolean isDrivetrainAimedAtHub() {
        return Math.abs(drivetrain.getHeading().minus(Field.getTranslationToHub().getAngle())
                .getDegrees()) < DRIVETRAIN_HEADING_THRESHOLD.in(Degrees);
    }

    private boolean isValidDistanceFromHub() {
        Distance dist = Field.getDistanceToHub();
        return dist.compareTo(MIN_DISTANCE_FROM_HUB) > 0 && dist.compareTo(MAX_DISTANCE_FROM_HUB) < 0;
    }

    /**
     * Drives to a point that is in the valid distance ring.
     * Targets the middle of the valid distance ring as to reach a valid pose faster
     */
    private DriveCommands.TranslationalVelocity driveToValidDistanceFromHub() {
        if (isValidDistanceFromHub())
            return DriveCommands.TranslationalVelocity.none();

        Distance middle = MAX_DISTANCE_FROM_HUB.plus(MIN_DISTANCE_FROM_HUB).div(2);
        return drivetrain.commands
                .driveToDistanceFromPoint(Field.HUB_CENTER_TRANSLATION::get, () -> middle).get();
    }

    private boolean isShooterAtVelocity(AngularVelocity velocity) {
        return Math.abs(shooter.getVelocity().in(RotationsPerSecond)
                - velocity.in(RotationsPerSecond)) < SHOOTER_VELOCITY_THRESHOLD.in(RotationsPerSecond);
    }

    public boolean hubShootRequirementsMet() {
        return isDrivetrainAimedAtHub() &&
                isValidDistanceFromHub() &&
                isShooterAtVelocity(getShooterVelocityForPosition()) &&
                Field.isHubActive() &&
                Field.inAllianceZone();
    }

    private boolean isDrivetrainAimedAtPassPoint() {
        return Math.abs(drivetrain.getHeading().minus(Field.getTranslationToPassPoint().getAngle())
                .getDegrees()) < DRIVETRAIN_HEADING_THRESHOLD.in(Degrees);
    }

    public boolean passRequirementsMet() {
        return isDrivetrainAimedAtPassPoint() &&
                isShooterAtVelocity(ShooterConstants.PASSING_OUTPUT) &&
                !Field.inAllianceZone();
    }

    /**
     * Command to aim and shoot into the hub.
     * Only shoots when the robot has the correct heading, the shooter has the
     * correct velocity, the robot is in the alliance zone, and the hub is active
     */
    private Command shootIntoHub() {
        return Commands.parallel(
                // Constantly sets the correct velocity for the shooter and heading for the
                // drivebase
                shooter.setVelocityCommand(this::getShooterVelocityForPosition),

                drivetrain.commands.drive(this::driveToValidDistanceFromHub,
                        drivetrain.commands.rotateToFacePoint(Field.HUB_CENTER_TRANSLATION::get)),

                // Shoots when all of the conditions are met.
                shooter.feeder.enableCommand().onlyWhile(this::hubShootRequirementsMet),
                spindexer.enableCommand().onlyWhile(this::hubShootRequirementsMet),

                // Set the `isCurrentlyShooting` variable
                Commands.run(() -> isCurrentlyShooting = hubShootRequirementsMet()))
                .until(() -> !Field.inAllianceZone());
    }

    /**
     * Command to shoot into the alliance zone, ensuring that the balls do not fall
     * into the hub (foul) or go outside of the field (foul)
     */
    private Command shootIntoAllianceZone() {
        return Commands.parallel(
                shooter.setVelocityCommand(ShooterConstants.PASSING_OUTPUT),
                // maybe add a translation to move past "hub line"
                drivetrain.commands.drive(null,
                        drivetrain.commands.rotateToFacePoint(Field::getTranslationOfPassPoint)),

                shooter.feeder.enableCommand().onlyWhile(this::passRequirementsMet),
                spindexer.enableCommand().onlyWhile(this::hubShootRequirementsMet),

                Commands.run(() -> isCurrentlyShooting = passRequirementsMet()));
    }

    /**
     * Main shooting command. Decides to pass or shoot into the hub based off the
     * robot's position on the field
     */
    public Command shoot() {
        return Commands.either(
                shootIntoHub(),
                shootIntoAllianceZone(),
                Field::inAllianceZone)
                .finallyDo(() -> isCurrentlyShooting = false);
    }

    /**
     * Publishes a trajectory to network tables with a line from the robot to it's
     * potential target
     */
    private void publishTargetLine() {
        Translation2d robotTranslation = drivetrain.getTranslation();

        // If the robot is in the alliance zone, aim at hub. Otherwise, aim at passing
        // point
        Translation2d targetTranslation;
        if (Field.inAllianceZone())
            targetTranslation = Field.HUB_CENTER_TRANSLATION.get();
        else
            targetTranslation = Field.getTranslationOfPassPoint();

        targetLinePublisher.set(new Translation2d[] { robotTranslation, targetTranslation });
    }

    @Override
    public void periodic() {
        publishTargetLine();
        SmartDashboard.putBoolean("hubShootRequirementsMet", hubShootRequirementsMet());
        SmartDashboard.putBoolean("isDrivetrainAimedAtPassPoint", isDrivetrainAimedAtPassPoint());
        SmartDashboard.putBoolean("arePassRequirementsMet", passRequirementsMet());
        SmartDashboard.putBoolean("isDrivetrainAimedAtHub", isDrivetrainAimedAtHub());
        SmartDashboard.putBoolean("isShooterAtVelocity", isShooterAtVelocity(getShooterVelocityForPosition()));
    }
}
