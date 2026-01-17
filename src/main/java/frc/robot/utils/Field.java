package frc.robot.utils;

import static edu.wpi.first.units.Units.Inches;

import java.util.Optional;

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

  public boolean hubactivated() {
    char hubactive = DriverStation.getGameSpecificMessage().charAt(0); // returns who is disabled first
    Optional<Alliance> alliance = DriverStation.getAlliance();
    double timeleft = DriverStation.getMatchTime();

    if ((timeleft < 30.0)/* endgame/auto */ || (timeleft > 130.0)/* Transition shift */) {
      return true;
    } 
    else {
      if ((hubactive == 'B' && alliance.get() == Alliance.Blue) || (hubactive == 'R' && alliance.get() == Alliance.Red)) {
        if ((timeleft > 105.0) || ((timeleft > 55.0) && (timeleft < 80.0))) {
          return false;
        }
        return true;
      } 
      else {
        if ((timeleft > 105.0) || ((timeleft > 55.0) && (timeleft < 80.0))) {
          return true;
        }
        return false;
      }
    }
  }

  /**
   * Trigger for if the robot is in the alliance zone corresponding to their
   * alliance. If no alliance has been set, defaults to red.
   */
  public static final Trigger inAllianceZone = new Trigger(() -> {
    Translation2d translation = RobotContainer.drivetrain.getTranslation();
    Rectangle2d allianceZone = ALLIANCE_ZONE.get();

    return allianceZone.contains(translation);
  });
}
