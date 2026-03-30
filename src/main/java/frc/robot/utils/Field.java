package frc.robot.utils;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.util.Arrays;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.button.Trigger;

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

    public static final double BUMP_MIDPOINT = Inches.of(62.35 + 73/2).in(Meters);
    public static final double TRENCH_MIDPOINT = Inches.of(50.35/2).in(Meters);


    public static final AllianceRelativeRectangle2d ALLIANCE_ZONE = AllianceRelativeRectangle2d
            .fromBlueRectangle(new Rectangle2d(Translation2d.kZero, new Translation2d(Inches.of(170), FIELD_WIDTH)));

    // Game Manual Section 5.4.
    public static final AllianceRelativeTranslation2d HUB_CENTER_TRANSLATION = AllianceRelativeTranslation2d
            .fromBlueTranslation(new Translation2d(
                    Inches.of(182.11),
                    FIELD_WIDTH.div(2)));

    public static final AllianceRelativeTranslation2d PASSING_TARGET_RIGHT_TRANSLATION = AllianceRelativeTranslation2d
            .fromBlueTranslation(new Translation2d(
                    Inches.of(90.78),
                    Inches.of(102.92)));

    // The Y-values are (FEILD_WIDTH / 2) +- 55.93
    public static final AllianceRelativeTranslation2d PASSING_TARGET_LEFT_TRANSLATION = AllianceRelativeTranslation2d
            .fromBlueTranslation(new Translation2d(
                    Inches.of(90.78),
                    Inches.of(214.78)));

    public static final AllianceRelativeRectangle2d PASSING_DEADZONE = AllianceRelativeRectangle2d
            .fromBlueRectangle(
                new Rectangle2d(
                        new Translation2d(
                                Field.FIELD_LENGTH.div(2), 
                                Field.FIELD_WIDTH.div(2).plus(Inches.of(50))), 
                        new Translation2d(
                                Inches.of(200), 
                                Field.FIELD_WIDTH.div(2).minus(Inches.of(50)))));

    public static final AllianceRelativeRectangle2d LOWER_TRENCH_ZONE = AllianceRelativeRectangle2d
            .fromBlueRectangle(
                new Rectangle2d(
                        new Translation2d(
                                Inches.of(0), 
                                Inches.of(0)), 
                        new Translation2d(
                                Field.FIELD_LENGTH, 
                                Meters.of(BUMP_MIDPOINT))));

        public static final AllianceRelativeRectangle2d UPPER_TRENCH_ZONE = AllianceRelativeRectangle2d
            .fromBlueRectangle(
                new Rectangle2d(
                        new Translation2d(
                                Inches.of(0), 
                                Field.FIELD_WIDTH), 
                        new Translation2d(
                                Field.FIELD_LENGTH, 
                                Field.FIELD_WIDTH.minus(Meters.of(BUMP_MIDPOINT)))));

    public static final AllianceRelativeRectangle2d LOWER_BUMP_ZONE = AllianceRelativeRectangle2d
            .fromBlueRectangle(
                new Rectangle2d(
                        new Translation2d(
                                Inches.of(0),
                                Meters.of(Field.BUMP_MIDPOINT)), 
                        new Translation2d(
                                Field.FIELD_LENGTH, 
                                Field.FIELD_WIDTH.div(2))));

    public static final AllianceRelativeRectangle2d UPPER_BUMP_ZONE = AllianceRelativeRectangle2d
            .fromBlueRectangle(
                new Rectangle2d(
                        new Translation2d(
                                Inches.of(0), 
                                Field.FIELD_WIDTH.minus(Meters.of(Field.BUMP_MIDPOINT))), 
                        new Translation2d(
                                Field.FIELD_LENGTH, 
                                Field.FIELD_WIDTH.div(2))));


    public static boolean isHubActive() {
        // https://docs.wpilib.org/en/stable/docs/yearly-overview/2026-game-data.html
        Alliance disabledFirst;

        if (!DriverStation.getGameSpecificMessage().equals("")) {
                disabledFirst = DriverStation.getGameSpecificMessage().charAt(0) == 'B' ? Alliance.Blue : Alliance.Red;
        }
        else {
                disabledFirst = DriverStation.getAlliance().orElse(Alliance.Red);
        }

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

    /**
     * Returns a Translation2d from the robot to the hub.
     */
    public static Translation2d getTranslationToHub() {
        return HUB_CENTER_TRANSLATION.get().minus(RobotPosition.getTranslation());
    }

    /**
     * Returns the robot's distance from the hub.
     */
    public static Distance getDistanceToHub() {
        return Meters.of(Field.getTranslationToHub().getNorm());
    }

    /**
     * Gets the translation of the nearest passing point
     */
    public static Translation2d getTranslationOfPassPoint() {
        Translation2d robotTranslation = RobotPosition.getTranslation();

        return robotTranslation.nearest(Arrays.asList(
                new Translation2d[] {
                        PASSING_TARGET_RIGHT_TRANSLATION.get(),
                        PASSING_TARGET_LEFT_TRANSLATION.get()
                }));
    }

    /**
     * Gets the translation from the robot to the nearest passing point
     */
    public static Translation2d getTranslationToPassPoint() {
        return getTranslationOfPassPoint().minus(RobotPosition.getTranslation());
    }

    /**
     * Returns the robot's distance to the nearest pass point.
     */
    public static Distance getDistanceToPassPoint() {
        return Meters.of(Field.getTranslationToPassPoint().getNorm());
    }

    /**
     * Boolean method for if the robot is in the alliance zone corresponding to
     * their alliance. If no alliance has been set, defaults to red.
     */
    public static boolean inAllianceZone() {
        Translation2d translation = RobotPosition.getTranslation();
        Rectangle2d allianceZone = ALLIANCE_ZONE.get();

        return allianceZone.contains(translation);
    }

    public static boolean isInPassingDeadzone() {
        Translation2d translation = RobotPosition.getTranslation();
        Rectangle2d deadzone = PASSING_DEADZONE.get();

        return deadzone.contains(translation);
    }

    public static boolean inBumpZone() {
        Translation2d translation = RobotPosition.getTranslation();
        Rectangle2d allianceZone1 = LOWER_BUMP_ZONE.get();
        Rectangle2d allianceZone2 = UPPER_BUMP_ZONE.get();

        return allianceZone1.contains(translation) || allianceZone2.contains(translation);
    }

    public static boolean inTrenchZone() {
        Translation2d translation = RobotPosition.getTranslation();
        Rectangle2d allianceZone1 = LOWER_TRENCH_ZONE.get();
        Rectangle2d allianceZone2 = UPPER_TRENCH_ZONE.get();

        return allianceZone1.contains(translation) || allianceZone2.contains(translation);
    }

    /**
     * Trigger for if the robot is in the alliance zone corresponding to their
     * alliance. If no alliance has been set, defaults to red.
     */
    public static final Trigger inAllianceZoneTrigger = new Trigger(Field::inAllianceZone);
}