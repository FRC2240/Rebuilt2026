package frc.robot.utils.logging;

import java.util.Arrays;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Distance;

public class VisualLogger {

  // For Points
  public static void publishPoint(String key, Translation2d trans) {
    key = "VisualLogger/" + key;
    StructPublisher<Translation2d> publisher = Register.getT2dPublisherObj(key);

    publisher.set(trans);
  }

  // For Lines
  public static void publishLine(String key, Translation2d A, Translation2d B) {
    key = "VisualLogger/" + key;
    StructArrayPublisher<Translation2d> arrayPublisher = Register.getT2dPublisherArrayObj(key);

    arrayPublisher.set(new Translation2d[] { A, B });
  }

  public static void publishPolygon(String key, Translation2d[] A) {
    key = "VisualLogger/" + key;

    Translation2d[] trans = Arrays.copyOf(A, A.length+1);
    trans[A.length] = trans[0];

    StructArrayPublisher<Translation2d> arrayPublisher = Register.getT2dPublisherArrayObj(key);

    arrayPublisher.set(trans);
  }

  public static void publishRect(String key, Rectangle2d rect) {
    key = "VisualLogger/" + key;

    Distance centerX = rect.getCenter().getMeasureX();
    Distance centerY = rect.getCenter().getMeasureY();

    Translation2d[] trans = new Translation2d[] {
      // 0, 0
      new Translation2d(centerX.minus(rect.getMeasureXWidth().div(2)), centerY.minus(rect.getMeasureYWidth().div(2))),
      // 0, 1
      new Translation2d(centerX.minus(rect.getMeasureXWidth().div(2)), centerY.plus(rect.getMeasureYWidth().div(2))),
      // 1, 1
      new Translation2d(centerX.plus(rect.getMeasureXWidth().div(2)), centerY.plus(rect.getMeasureYWidth().div(2))),
      // 1, 0
      new Translation2d(centerX.plus(rect.getMeasureXWidth().div(2)), centerY.minus(rect.getMeasureYWidth().div(2))),
      // Back to 0, 0
      new Translation2d(centerX.minus(rect.getMeasureXWidth().div(2)), centerY.minus(rect.getMeasureYWidth().div(2))),

    };

    StructArrayPublisher<Translation2d> arrayPublisher = Register.getT2dPublisherArrayObj(key);

    arrayPublisher.set(trans);
  }
}