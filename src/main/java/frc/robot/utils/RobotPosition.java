package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class RobotPosition {
    private final Drivetrain drivetrain;
    private static RobotPosition instance;

    // Private constructor to regulate instance creation
    private RobotPosition(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    /**
     * Initializes the singelton. This needs to be run before using any other
     * functions in this class.
     */
    public static void init(Drivetrain drivetrain) {
        if (instance == null)
            instance = new RobotPosition(drivetrain);
    }

    /**
     * Gets the estimated Pose2d of the robot on the field
     */
    public static Pose2d getPose() {
        return instance.drivetrain.getPose();
    }

    /**
     * Gets the robot's estimated translation on the field.
     */
    public static Translation2d getTranslation() {
        return instance.drivetrain.getTranslation();
    }

    /**
     * Gets the robot's estimated heading (Where it is facing)
     */
    public static Rotation2d getHeading() {
        return instance.drivetrain.getHeading();
    }
}