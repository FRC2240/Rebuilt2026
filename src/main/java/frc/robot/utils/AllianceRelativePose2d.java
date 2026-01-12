package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class AllianceRelativePose2d {
    public static Pose2d flipPose(Pose2d pose) {
        return pose.rotateAround(Field.FIELD_CENTER, Rotation2d.k180deg);
    }

    private final Pose2d bluePose;
    private final Pose2d redPose;

    private AllianceRelativePose2d(Pose2d bluePose, Pose2d redPose) {
        this.bluePose = bluePose;
        this.redPose = redPose;
    }

    public static AllianceRelativePose2d fromBluePose(Pose2d pose) {
        return new AllianceRelativePose2d(pose, flipPose(pose));
    }

    public static AllianceRelativePose2d fromRedPose(Pose2d pose) {
        return new AllianceRelativePose2d(flipPose(pose), pose);
    }

    public Pose2d get() {
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Red);
        if (alliance == Alliance.Red)
            return this.redPose;
        else
            return this.bluePose;
    }

    public Pose2d getBluePose() {
        return bluePose;
    }

    public Pose2d getRedPose() {
        return redPose;
    }
}
