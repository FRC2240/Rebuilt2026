package frc.robot.utils;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.shooter.Shooter;

public class ShootingController {
    private static final AngularVelocity SHOOTER_VELOCITY_THRESHOLD = RotationsPerSecond.of(1);
    private static final Angle DRIVETRAIN_HEADING_THRESHOLD = Degrees.of(5);

    private final Drivetrain drivetrain;
    private final Shooter shooter;

    // Inches to Rotations Per Second
    private final InterpolatingDoubleTreeMap distanceToVelocityMap = new InterpolatingDoubleTreeMap();

    public ShootingController(Drivetrain drivetrain, Shooter shooter) {
        this.drivetrain = drivetrain;
        this.shooter = shooter;

        // Set values for the tree map
        distanceToVelocityMap.put(0., 0.);
        distanceToVelocityMap.put(10., 10.);
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

                // Shoots when all of the conditions are met
                shooter.indexer.enableCommand().onlyWhile(() -> isDrivetrainAimedAtHub() &&
                        isShooterAtVelocity(getShooterVelocityForPosition()) &&
                        Field.inAllianceZone() &&
                        Field.isHubActive()));
    }

    /**
     * Command to shoot into the alliance zone, ensuring that the balls do not fall
     * into the hub (foul) or go outside of the field (foul)
     */
    private Command shootIntoAllianceZone() {
        return Commands.none(); // TODO
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
        });
    }
}
