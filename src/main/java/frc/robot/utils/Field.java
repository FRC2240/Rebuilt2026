package frc.robot.utils;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;

/*
 * Holds constants and methods related to the field.
 */
public class Field {
    public static final AprilTagFieldLayout APRIL_TAG_LAYOUT = AprilTagFieldLayout
            .loadField(AprilTagFields.k2026RebuiltWelded);

    // https://firstfrc.blob.core.windows.net/frc2026/FieldAssets/2026-field-dimension-dwgs.pdf
    public static final Distance FIELD_LENGTH = Inches.of(651.22); // X
    public static final Distance FIELD_WIDTH = Inches.of(317.69); // Y
    public static final Translation2d FIELD_CENTER = new Translation2d(FIELD_LENGTH.div(2), FIELD_WIDTH.div(2));

    public static final AllianceRelativeRectangle2d ALLIANCE_ZONE = AllianceRelativeRectangle2d
            .fromBlueRectangle(new Rectangle2d(Translation2d.kZero, new Translation2d(Inches.of(170), FIELD_WIDTH)));

    // Game Manual Section 5.4.
    public static final AllianceRelativeTranslation2d HUB_CENTER_TRANSLATION = AllianceRelativeTranslation2d
            .fromBlueTranslation(new Translation2d(
                    Inches.of(182.11),
                    FIELD_WIDTH.div(2)));

    public static boolean isHubActive() {
        // https://docs.wpilib.org/en/stable/docs/yearly-overview/2026-game-data.html
        Alliance disabledFirst = DriverStation.getGameSpecificMessage().charAt(0) == 'B' ? Alliance.Blue : Alliance.Red;
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Red);
        boolean isInactiveFirst = disabledFirst == alliance;

        double timeleft = DriverStation.getMatchTime();
        // Both hubs are active in auto, in transistion shift, and in end game
        boolean areBothActive = DriverStation.isAutonomous() || timeleft > 130.0 || timeleft < 30.0;
        // The team that is inactive first is active in shift 2 and 4
        boolean isFirstInactiveActive = (timeleft < 105.0 && timeleft > 80.0) || (timeleft < 55.0 && timeleft > 30.0);
        // The team that is inactive second is active in shift 1 and 3
        boolean isSecondInactiveActive = (timeleft < 130.0 && timeleft > 105.0) || (timeleft < 80.0 && timeleft > 55.0);

        return areBothActive || (isInactiveFirst && isFirstInactiveActive)
                || (!isInactiveFirst && isSecondInactiveActive);
    }

    public static Translation2d getTranslationToHub() {
        return HUB_CENTER_TRANSLATION.get().minus(RobotContainer.drivetrain.getTranslation());
    }

    public static Distance getDistanceToHub() {
        return Meters.of(Field.getTranslationToHub().getNorm());
    }

    /**
     * Boolean method for if the robot is in the alliance zone corresponding to
     * their alliance. If no alliance has been set, defaults to red.
     */
    public static boolean inAllianceZone() {
        Translation2d translation = RobotContainer.drivetrain.getTranslation();
        Rectangle2d allianceZone = ALLIANCE_ZONE.get();

        return allianceZone.contains(translation);
    }

    /**
     * Trigger for if the robot is in the alliance zone corresponding to their
     * alliance. If no alliance has been set, defaults to red.
     */
    public static final Trigger inAllianceZoneTrigger = new Trigger(Field::inAllianceZone);
}
