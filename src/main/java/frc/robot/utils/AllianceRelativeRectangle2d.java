package frc.robot.utils;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class AllianceRelativeRectangle2d {
    private final Rectangle2d blueRectangle;
    private final Rectangle2d redRectangle;

    public static Rectangle2d flipRectangle(Rectangle2d rectangle) {
        Pose2d newCenter = AllianceRelativePose2d.flipPose(rectangle.getCenter());
        return new Rectangle2d(newCenter, rectangle.getXWidth(), rectangle.getYWidth());
    }

    private AllianceRelativeRectangle2d(Rectangle2d blueRectangle, Rectangle2d redRectangle) {
        this.blueRectangle = blueRectangle;
        this.redRectangle = redRectangle;
    }

    public static AllianceRelativeRectangle2d fromBlueRectangle(Rectangle2d rectangle) {
        System.out.println(Field.FIELD_LENGTH.in(Meters));
        System.out.println(Field.FIELD_WIDTH.in(Meters));
        return new AllianceRelativeRectangle2d(rectangle, flipRectangle(rectangle));
    }

    public static AllianceRelativeRectangle2d fromRedRectangle(Rectangle2d rectangle) {
        return new AllianceRelativeRectangle2d(flipRectangle(rectangle), rectangle);
    }

    public Rectangle2d get() {
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Red);
        if (alliance == Alliance.Red)
            return this.redRectangle;
        else
            return this.blueRectangle;
    }
}
