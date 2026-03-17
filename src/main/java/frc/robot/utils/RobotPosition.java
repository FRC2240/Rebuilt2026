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

    public static void init(Drivetrain drivetrain) {
        if (instance == null)
            instance = new RobotPosition(drivetrain);
    }

    public static Pose2d getPose() {
        return instance.drivetrain.getPose();
    }

    public static Translation2d getTranslation() {
        return instance.drivetrain.getTranslation();
    }

    public static Rotation2d getHeading() {
        return instance.drivetrain.getHeading();
    }
}