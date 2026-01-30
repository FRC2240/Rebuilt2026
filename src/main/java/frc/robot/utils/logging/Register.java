package frc.robot.utils.logging;

import java.util.HashMap;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;

public class Register<T> {
    public static HashMap<String, StructPublisher<Translation2d>> map = new HashMap<>();
    public static HashMap<String, StructArrayPublisher<Translation2d>> arrayMap = new HashMap<>();

    public static void registerT2d(String key) {
        StructPublisher<Translation2d> publisher = NetworkTableInstance.getDefault()
        .getStructTopic(key, Translation2d.struct).publish();

        Register.map.put(key, publisher);
    }

    public static void registerT2dArray(String key) {
        StructArrayPublisher<Translation2d> arrayPublisher = NetworkTableInstance.getDefault()
      .getStructArrayTopic((key), Translation2d.struct).publish();

        Register.arrayMap.put(key, arrayPublisher);
    }

    public static StructPublisher<Translation2d> getT2dPublisherObj(String key) {
        return Register.map.get(key);
    }

    public static StructArrayPublisher<Translation2d> getT2dPublisherArrayObj(String key) {
        return Register.arrayMap.get(key);
    }
}
