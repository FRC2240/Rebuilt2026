package frc.robot.utils.logging;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;

public class VisualLogger {

  // For Points
  public static void publishTranslation2d(String key, Translation2d trans) {
    StructPublisher<Translation2d> publisher = Register.getT2dPublisherObj(key);

    publisher.set(trans);
  }

  // For Lines
  public static void publishTranslation2dArray(String key, Translation2d A, Translation2d B) {
    StructArrayPublisher<Translation2d> arrayPublisher = Register.getT2dPublisherArrayObj(key);

    arrayPublisher.set(new Translation2d[] { A, B });
  }
}
