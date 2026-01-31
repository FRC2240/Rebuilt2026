package frc.robot.subsystems.target;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;

public class TargetTracking {
    public static StructArrayPublisher<Translation2d> arrayPublisher = NetworkTableInstance.getDefault()
                .getStructArrayTopic(("Target_Line"), Translation2d.struct).publish();

    public static void publish(Translation2d[] arr) {
        arrayPublisher.set(arr);
    }
}
