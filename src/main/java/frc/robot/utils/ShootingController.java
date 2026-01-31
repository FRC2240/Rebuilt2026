package frc.robot.utils;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.target.TargetTracking;
import frc.robot.subsystems.shooter.ShooterConstants;

public class ShootingController {
    private static final AngularVelocity SHOOTER_VELOCITY_THRESHOLD = RotationsPerSecond.of(1);
    private static final Angle DRIVETRAIN_HEADING_THRESHOLD = Degrees.of(5);

    private final Drivetrain drivetrain;
    private final Shooter shooter;
    private final Spindexer spindexer;
    private final TargetTracking tracker;

    // Inches to Rotations Per Second
    private final InterpolatingDoubleTreeMap distanceToVelocityMap = new InterpolatingDoubleTreeMap();

    private boolean isCurrentlyShooting = false;

    public ShootingController(Drivetrain drivetrain, Shooter shooter, Spindexer spindexer, TargetTracking tracker) {
        this.drivetrain = drivetrain;
        this.shooter = shooter;
        this.spindexer = spindexer;
        this.tracker = tracker;

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

    private boolean isShooterAtVelocity(AngularVelocity velocity) {
        return Math.abs(shooter.getVelocity().in(RotationsPerSecond)
                - velocity.in(RotationsPerSecond)) < SHOOTER_VELOCITY_THRESHOLD.in(RotationsPerSecond);
    }

    public boolean hubShootRequirementsMet() {
        return isDrivetrainAimedAtHub() &&
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
                drivetrain.commands.drive(null, drivetrain.commands.rotateToAimAtHub()),

                // Shoots when all of the conditions are met.
                shooter.indexer.enableCommand().onlyWhile(this::hubShootRequirementsMet),
                spindexer.enableCommand().onlyWhile (this::hubShootRequirementsMet),

                // Set the `isCurrentlyShooting` variable
                Commands.run(() -> isCurrentlyShooting = hubShootRequirementsMet()));
    }

    /**
     * Command to shoot into the alliance zone, ensuring that the balls do not fall
     * into the hub (foul) or go outside of the field (foul)
     */
    private Command shootIntoAllianceZone() {
        return Commands.parallel(
            shooter.setVelocityCommand(ShooterConstants.PASSING_OUTPUT),
            // maybe add a translation to move past "hub line"
            drivetrain.commands.drive(null, drivetrain.commands.rotateToPass()),

            shooter.indexer.enableCommand().onlyWhile(this::passRequirementsMet),
            spindexer.enableCommand().onlyWhile (this::hubShootRequirementsMet),


            Commands.run(() -> isCurrentlyShooting = passRequirementsMet())
        );
    }

    /**
     * Main shooting command. Decides to pass or shoot into the hub based off the
     * robot's position on the field
     */
    public Command shoot() {
        return Commands.deferredProxy(() -> {
            if (Field.inAllianceZone())
                return shootIntoHub();
            else
                return shootIntoAllianceZone();
        }).finallyDo(() -> isCurrentlyShooting = false);
    }

    public static void publishCurrentTarget() {
        Supplier<Translation2d> robotTranslation = RobotContainer.drivetrain::getTranslation;
        
        if (Field.inAllianceZone()) {
            TargetTracking.publish(new Translation2d[] {robotTranslation.get(), Field.HUB_CENTER_TRANSLATION.get()});
            return;
        }
        
        double dist_right = robotTranslation.get().getDistance(Field.PASSING_TARGET_RIGHT_TRANSLATION.get());
        double dist_left = robotTranslation.get().getDistance(Field.PASSING_TARGET_LEFT_TRANSLATION.get());

        if (robotTranslation.get() != null) {
            if (dist_right > dist_left) {
                TargetTracking.publish(new Translation2d[] {robotTranslation.get(), Field.PASSING_TARGET_LEFT_TRANSLATION.get()});
                return;
            }
            TargetTracking.publish(new Translation2d[] {robotTranslation.get(), Field.PASSING_TARGET_RIGHT_TRANSLATION.get()});
            return;
        }

        // This means the Robot's translation was null while outside of Alliance Zone
        TargetTracking.publish(new Translation2d[] {robotTranslation.get(), new Translation2d(0, 0)});
    }

    
}
