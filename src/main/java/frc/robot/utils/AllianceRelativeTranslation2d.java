package frc.robot.utils;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class AllianceRelativeTranslation2d {
    /**
     * Utility function to flip a Translation2d on the field.
     * (Rotates it 180 degrees about the center point of the field)
     */
    public static Translation2d flipTranslation(Translation2d translation) {
        return translation.rotateAround(Field.FIELD_CENTER, Rotation2d.k180deg);
    }

    private final Translation2d blueTranslation;
    private final Translation2d redTranslation;

    private AllianceRelativeTranslation2d(Translation2d blueTranslation, Translation2d redTranslation) {
        this.blueTranslation = blueTranslation;
        this.redTranslation = redTranslation;
    }

    /**
     * Creates an AllianceRelativeTranslation2d from a translation in the blue
     * alliance.
     */
    public static AllianceRelativeTranslation2d fromBlueTranslation(Translation2d translation) {
        return new AllianceRelativeTranslation2d(translation, flipTranslation(translation));
    }

    /**
     * Creates an AllianceRelativeTranslation2d from a translation in the red alliance.
     */
    public static AllianceRelativeTranslation2d fromRedTranslation(Translation2d translation) {
        return new AllianceRelativeTranslation2d(flipTranslation(translation), translation);
    }

    /**
     * Gets the Translation2d corresponding to the current alliance. If there is no
     * alliance, defaults to red.
     */
    public Translation2d get() {
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Red);
        if (alliance == Alliance.Red)
            return this.redTranslation;
        else
            return this.blueTranslation;
    }

    /**
     * Gets the Translation2d corresponding to the blue alliance
     */
    public Translation2d getBlueTranslation() {
        return blueTranslation;
    }

    /**
     * Gets the Translation2d corresponding to the red alliance
     */
    public Translation2d getRedTranslation() {
        return redTranslation;
    }
}