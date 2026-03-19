package frc.robot.utils.logging;

import java.util.Arrays;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;

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
}