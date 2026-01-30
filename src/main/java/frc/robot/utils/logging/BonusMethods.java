package frc.robot.utils.logging;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.RobotContainer;
import frc.robot.utils.Field;

public class BonusMethods {
    public static Translation2d BM_getTargetTranslation() {
        Supplier<Translation2d> robotTranslation = RobotContainer.drivetrain::getTranslation;

        double dist_right = robotTranslation.get().getDistance(Field.PASSING_TARGET_RIGHT_TRANSLATION.get());
        double dist_left = robotTranslation.get().getDistance(Field.PASSING_TARGET_LEFT_TRANSLATION.get());

        if (Field.inAllianceZone()) {
            return Field.HUB_CENTER_TRANSLATION.get();
        }

        if (robotTranslation.get() != null) {
            if (dist_right > dist_left) {
                return Field.PASSING_TARGET_LEFT_TRANSLATION.get();
            }
            return Field.PASSING_TARGET_RIGHT_TRANSLATION.get();
        }

        return new Translation2d(1, 1);
    }
}
