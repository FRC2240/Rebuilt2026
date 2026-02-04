package frc.robot.utils.logging;

import static edu.wpi.first.units.Units.Inches;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.RobotContainer;
import frc.robot.utils.Field;
import frc.robot.utils.ShootingController;

public class LogDataMethods {
    public static Translation2d shooterGetTargetLogic() {
        if (Field.inAllianceZone()) return Field.HUB_CENTER_TRANSLATION.get();
        
        Supplier<Translation2d> robotTranslation = RobotContainer.drivetrain::getTranslation;

        double dist_right = robotTranslation.get().getDistance(Field.PASSING_TARGET_RIGHT_TRANSLATION.get());
        double dist_left = robotTranslation.get().getDistance(Field.PASSING_TARGET_LEFT_TRANSLATION.get());

        if (robotTranslation.get() != null) {
            if (dist_right > dist_left) {
                return Field.PASSING_TARGET_LEFT_TRANSLATION.get();
            }
            return Field.PASSING_TARGET_RIGHT_TRANSLATION.get();
        }

        return new Translation2d(1, 1);
    }

    double SLOPE = (158.85-108.85)/(325.61-200);

    public static int isInDeadZone() {
        Supplier<Translation2d> robotPose = RobotContainer.drivetrain::getTranslation;

        if (!isBetweenX(robotPose.get()) || !isBetweenY(robotPose.get())) return 0;

        if (((158.85-108.85)/(325.61-200)) > getSlope(robotPose.get(), shooterGetTargetLogic())) return 0;

        return 1;
    }

    public static double getSlope(Translation2d A, Translation2d B) {
        double x  = B.getMeasureX().baseUnitMagnitude() - A.getMeasureX().baseUnitMagnitude();
        double y  = B.getMeasureY().baseUnitMagnitude() - A.getMeasureY().baseUnitMagnitude();

        return Math.abs(y/x);
    }

    public static boolean isBetweenX(Translation2d pose) {
        return (pose.getMeasureX().gte(Inches.of(200)) && pose.getMeasureX().lte(Inches.of(325.61)));
    }

    public static boolean isBetweenY(Translation2d pose) {
        return (pose.getMeasureY().gte(Inches.of(108.85)) && pose.getMeasureY().lte(Inches.of(208.85)));
    } 
}
