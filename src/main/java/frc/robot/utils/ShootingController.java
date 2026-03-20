package frc.robot.utils;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drivetrain.DriveCommands;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.spindexer.Spindexer;

public class ShootingController extends SubsystemBase {
    private static AngularVelocity SHOOTER_VELOCITY_THRESHOLD = RotationsPerSecond.of(4);
    private static final Angle DRIVETRAIN_HEADING_THRESHOLD = Degrees.of(3);

    private static final LinearVelocity DRIVETRAIN_STILL_LINEAR_TOLERANCE = InchesPerSecond.of(5);
    private static final AngularVelocity DRIVETRAIN_STILL_ANGULAR_TOLERANCE = DegreesPerSecond.of(10);

    private static final Distance MIN_DISTANCE_FROM_HUB = Inches.of(82);
    private static final Distance NORM_DISTANCE_FROM_HUB = Inches.of(90);
    private static final Distance MAX_DISTANCE_FROM_HUB = Inches.of(196);

    private final Drivetrain drivetrain;
    private final Shooter shooter;
    private final Spindexer spindexer;
    private final Intake intake;

    // Inches to Rotations Per Second
    private final InterpolatingDoubleTreeMap hubDistanceToVelocityMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap passDistanceToVelocityMap = new InterpolatingDoubleTreeMap();

    private boolean isCurrentlyShooting = false;

    public static StructArrayPublisher<Translation2d> targetLinePublisher = NetworkTableInstance.getDefault()
            .getStructArrayTopic(("Target_Line"), Translation2d.struct).publish();

    public ShootingController(Drivetrain drivetrain, Shooter shooter, Spindexer spindexer, Intake intake) {
        this.drivetrain = drivetrain;
        this.shooter = shooter;
        this.spindexer = spindexer;
        this.intake = intake;

        // Set values for the tree map
        hubDistanceToVelocityMap.put(76., 51.25);
        hubDistanceToVelocityMap.put(92., 51.5);
        hubDistanceToVelocityMap.put(105.7, 54.);
        hubDistanceToVelocityMap.put(118.1, 56.);
        hubDistanceToVelocityMap.put(125.5, 57.);
        hubDistanceToVelocityMap.put(132., 59.);
        hubDistanceToVelocityMap.put(150.2, 63.);
        hubDistanceToVelocityMap.put(163.1, 65.);
        hubDistanceToVelocityMap.put(175., 68.);
        hubDistanceToVelocityMap.put(194.7, 69.);

        
        passDistanceToVelocityMap.put(65., 30.);
        passDistanceToVelocityMap.put(83., 35.);
        passDistanceToVelocityMap.put(119., 45.);
        passDistanceToVelocityMap.put(136.,50.);
        passDistanceToVelocityMap.put(149.,55.);
        passDistanceToVelocityMap.put(173.,60.);
        passDistanceToVelocityMap.put(209.,65.);
        passDistanceToVelocityMap.put(243.,70.);
        passDistanceToVelocityMap.put(281.,75.);
        passDistanceToVelocityMap.put(314.,80.);

        SmartDashboard.putNumber("Shooter velocity threshold", SHOOTER_VELOCITY_THRESHOLD.in(RotationsPerSecond));
    }

    public boolean isShooting() {
        return isCurrentlyShooting;
    }

    private AngularVelocity getShooterHubVelocityForPosition() {
        // This is abstracted to it's own method to avoid problems if switching units in
        // the tree map
        return RotationsPerSecond.of(hubDistanceToVelocityMap.get(Field.getDistanceToHub().in(Inches)));
    }

    private AngularVelocity getShooterPassVelocityForPosition() {
        // This is abstracted to it's own method to avoid problems if switching units in
        // the tree map
        return RotationsPerSecond.of(passDistanceToVelocityMap.get(Field.getDistanceToPassPoint().in(Inches)));
    }

    // These boolean functions are in this class instead of their respective
    // subsystems. This is because this class holds the constants such as tolerances
    // for aiming and shooting so holding the methods here avoids a complex web of
    // imports to and from subsystems. TLDR: Reduces complexity

    private boolean isDrivetrainAimedAtHub() {
        return Math.abs(Field.getTranslationToHub().getAngle().minus(drivetrain.getHeading())
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
        return drivetrain.commands
                .driveToDistanceFromPoint(Field.HUB_CENTER_TRANSLATION::get, () -> NORM_DISTANCE_FROM_HUB).get();
    }

    private boolean isShooterAtVelocity(AngularVelocity velocity) {
        return Math.abs(shooter.getVelocity().in(RotationsPerSecond)
                - velocity.in(RotationsPerSecond)) < SHOOTER_VELOCITY_THRESHOLD.in(RotationsPerSecond);
    }

    public boolean hubShootRequirementsMet() {
        return isDrivetrainAimedAtHub() &&
                isValidDistanceFromHub() &&
                (isShooterAtVelocity(getShooterHubVelocityForPosition()) || RobotBase.isSimulation()) &&
                // Field.isHubActive() &&
                Field.inAllianceZone();
    }

    /**
     * Returns true if the robot is reasonably still
     */
    public boolean isRobotStill() {
        ChassisSpeeds speeds = drivetrain.getState().Speeds;
        LinearVelocity linearVelocity = MetersPerSecond
                .of(Math.sqrt(Math.pow(speeds.vxMetersPerSecond, 2) + Math.pow(speeds.vyMetersPerSecond, 2)));

        return linearVelocity.isNear(MetersPerSecond.of(0), DRIVETRAIN_STILL_LINEAR_TOLERANCE) &&
                RadiansPerSecond.of(speeds.omegaRadiansPerSecond).isNear(DegreesPerSecond.of(0),
                        DRIVETRAIN_STILL_ANGULAR_TOLERANCE);
    }

    private boolean isDrivetrainAimedAtPassPoint() {
        return Math.abs(drivetrain.getHeading().minus(Field.getTranslationToPassPoint().getAngle())
                .getDegrees()) < DRIVETRAIN_HEADING_THRESHOLD.in(Degrees);
    }

    public boolean passRequirementsMet() {
        return isDrivetrainAimedAtPassPoint() &&
                !Field.isInPassingDeadzone() &&
                isShooterAtVelocity(getShooterPassVelocityForPosition()) &&
                !Field.inAllianceZone();
    }

    /**
     * Command to aim and shoot into the hub.
     * Only shoots when the robot has the correct heading, the shooter has the
     * correct velocity, the robot is in the alliance zone, and the hub is active
     */
    public Command shootIntoHub() {
        return Commands.parallel(
                // Constantly sets the correct velocity for the shooter and heading for the
                // drivebase
                Commands.waitUntil(this::isDrivetrainAimedAtHub).andThen(shooter.setVelocityCommand(this::getShooterHubVelocityForPosition)),

                drivetrain.commands.drive(this::driveToValidDistanceFromHub, drivetrain.commands.rotateToFacePoint(Field.HUB_CENTER_TRANSLATION::get)),

                BetterCommands.runWhen(feed(), this::hubShootRequirementsMet),
                
                intake.pivot.rampCommand(),

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
                shooter.setVelocityCommand(this::getShooterPassVelocityForPosition),
                drivetrain.commands.drive(drivetrain.commands.driveWithJoystick(),
                        drivetrain.commands.rotateToFacePoint(Field::getTranslationOfPassPoint)),

                BetterCommands.runWhen(feed(), this::passRequirementsMet),
                intake.pivot.rampCommand(),

                Commands.run(() -> isCurrentlyShooting = passRequirementsMet()));
    }

    /**
     * Command to enable the spindexer and the feeder at once
     */
    public Command feed() {
        return Commands.parallel(
                shooter.feeder.enableCommand(),
                spindexer.enableCommand(),
                //intake.pivot.rampCommand(),
                intake.enableIntakeCommand());
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
                .andThen(intake.enableIntakeCommand())
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
        SmartDashboard.putBoolean("ShootingConditions/isInAllianceZone", Field.inAllianceZone());
        SmartDashboard.putNumber("ShootingConditions/Inches from Hub", Field.getDistanceToHub().in(Inches));

        SmartDashboard.putBoolean("ShootingConditions/Hub/hubShootRequirementsMet", hubShootRequirementsMet());
        SmartDashboard.putBoolean("ShootingConditions/Hub/isDrivetrainAimedAtHub", isDrivetrainAimedAtHub());
        SmartDashboard.putBoolean("ShootingConditions/Hub/isValidDistanceFromHub", isValidDistanceFromHub());
        SmartDashboard.putBoolean("ShootingConditions/Hub/isShooterAtVelocity",
                isShooterAtVelocity(getShooterHubVelocityForPosition()));
        SmartDashboard.putNumber("ShootingConditions/Hub/desired flywheel velocity",
                getShooterHubVelocityForPosition().in(RotationsPerSecond));
        SmartDashboard.putNumber("ShootingConditions/Hub/real flywheel velocity", 
                shooter.getVelocity().in(RotationsPerSecond));
        SmartDashboard.putBoolean("ShootingConditions/Hub/isStill", isRobotStill());

        SmartDashboard.putBoolean("ShootingConditions/Pass/isDrivetrainAimedAtPassPoint",
                isDrivetrainAimedAtPassPoint());
        SmartDashboard.putBoolean("ShootingConditions/Pass/inPassingDeadzone", Field.isInPassingDeadzone());
        SmartDashboard.putBoolean("ShootingConditions/Pass/isShooterAtVelocity",
                isShooterAtVelocity(getShooterPassVelocityForPosition()));
        SmartDashboard.putNumber("ShootingConditions/Pass/desired flywheel velocity",
                getShooterPassVelocityForPosition().in(RotationsPerSecond));

        SHOOTER_VELOCITY_THRESHOLD = RotationsPerSecond.of(SmartDashboard.getNumber("Shooter velocity threshold", SHOOTER_VELOCITY_THRESHOLD.in(RotationsPerSecond)));
    }
}
