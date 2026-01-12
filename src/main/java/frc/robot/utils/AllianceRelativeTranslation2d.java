package frc.robot.utils;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class AllianceRelativeTranslation2d {
    public static Translation2d flipTranslation(Translation2d translation) {
        return translation.rotateAround(Field.FIELD_CENTER, Rotation2d.k180deg);
    }

    private final Translation2d blueTranslation;
    private final Translation2d redTranslation;

    private AllianceRelativeTranslation2d(Translation2d blueTranslation, Translation2d redTranslation) {
        this.blueTranslation = blueTranslation;
        this.redTranslation = redTranslation;
    }

    public static AllianceRelativeTranslation2d fromBlueTranslation(Translation2d translation) {
        return new AllianceRelativeTranslation2d(translation, flipTranslation(translation));
    }

    public static AllianceRelativeTranslation2d fromRedTranslation(Translation2d translation) {
        return new AllianceRelativeTranslation2d(flipTranslation(translation), translation);
    }

    public Translation2d get() {
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Red);
        if (alliance == Alliance.Red)
            return this.redTranslation;
        else
            return this.blueTranslation;
    }

    public Translation2d getBlueTranslation() {
        return blueTranslation;
    }

    public Translation2d getRedTranslation() {
        return redTranslation;
    }
}
