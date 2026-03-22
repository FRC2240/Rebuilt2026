package frc.robot.utils.BetterAutos;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.utils.AllianceRelativeTranslation2d;

public class AutoConstants {
        public static final double WAITTIME = 5;

    public static final AllianceRelativeTranslation2d RIGHT_TRENCH = AllianceRelativeTranslation2d
            .fromBlueTranslation(new Translation2d(
                    Inches.of(182.11),
                    Inches.of(100)));
    
    public static final AllianceRelativeTranslation2d RIGHT_BUMP = AllianceRelativeTranslation2d
            .fromBlueTranslation(new Translation2d(
                    Inches.of(182.11),
                    Inches.of(100)));

    public static final AllianceRelativeTranslation2d HUB = AllianceRelativeTranslation2d
            .fromBlueTranslation(new Translation2d(
                    Inches.of(182.11),
                    Inches.of(100)));

    public static final AllianceRelativeTranslation2d LEFT_TRENCH = AllianceRelativeTranslation2d
            .fromBlueTranslation(new Translation2d(
                    Inches.of(182.11),
                    Inches.of(100)));

    public static final AllianceRelativeTranslation2d LEFT_BUMP = AllianceRelativeTranslation2d
            .fromBlueTranslation(new Translation2d(
                    Inches.of(182.11),
                    Inches.of(100)));
    
}
