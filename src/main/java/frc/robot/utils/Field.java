package frc.robot.utils;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;

public class Field {
    public static AprilTagFieldLayout APRIL_TAG_LAYOUT = AprilTagFieldLayout
            .loadField(AprilTagFields.k2025ReefscapeWelded); // TODO: Change when released

    // https://firstfrc.blob.core.windows.net/frc2026/FieldAssets/2026-field-dimension-dwgs.pdf
    public static Distance FIELD_LENGTH = Inches.of(651.22); // X
    public static Distance FIELD_WIDTH = Inches.of(317.69); // Y
    public static Translation2d FIELD_CENTER = new Translation2d(FIELD_LENGTH.div(2), FIELD_WIDTH.div(2));

    public static AllianceRelativeRectangle2d ALLIANCE_ZONE = AllianceRelativeRectangle2d
            .fromBlueRectangle(new Rectangle2d(Translation2d.kZero, new Translation2d(Inches.of(170), FIELD_WIDTH)));

    // Game Manual Section 5.4.
    public static AllianceRelativeTranslation2d HUB_CENTER_TRANSLATION = AllianceRelativeTranslation2d
            .fromBlueTranslation(new Translation2d(
                    Inches.of(182.11),
                    FIELD_WIDTH.div(2)));

    public static Trigger inAllianceZone = new Trigger(() -> {
        Translation2d translation = RobotContainer.drivetrain.getTranslation();
        Rectangle2d allianceZone = ALLIANCE_ZONE.get();

        return allianceZone.contains(translation);
    });
}
