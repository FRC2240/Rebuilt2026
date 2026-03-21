package frc.robot.utils;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class AllianceRelativeRectangle2d {
    private final Rectangle2d blueRectangle;
    private final Rectangle2d redRectangle;

    /** 
     * Utility function to flip a rectangle on the field.
     * (Rotates it 180 degrees about the center point of the field)
     */
    public static Rectangle2d flipRectangle(Rectangle2d rectangle) {
        Pose2d newCenter = AllianceRelativePose2d.flipPose(rectangle.getCenter());
        return new Rectangle2d(newCenter, rectangle.getXWidth(), rectangle.getYWidth());
    }

    private AllianceRelativeRectangle2d(Rectangle2d blueRectangle, Rectangle2d redRectangle) {
        this.blueRectangle = blueRectangle;
        this.redRectangle = redRectangle;
    }

    /**
     * Creates an AllianceRelativeRectangle2d from a position in the blue
     * alliance.
     */
    public static AllianceRelativeRectangle2d fromBlueRectangle(Rectangle2d rectangle) {
        System.out.println(Field.FIELD_LENGTH.in(Meters));
        System.out.println(Field.FIELD_WIDTH.in(Meters));
        return new AllianceRelativeRectangle2d(rectangle, flipRectangle(rectangle));
    }

    /**
     * Creates an AllianceRelativeRectangle2d from a position in the red
     * alliance.
     */
    public static AllianceRelativeRectangle2d fromRedRectangle(Rectangle2d rectangle) {
        return new AllianceRelativeRectangle2d(flipRectangle(rectangle), rectangle);
    }

    /**
     * Gets the Rectangle2d corresponding to the current alliance. If there is no
     * alliance, defaults to red.
     */
    public Rectangle2d get() {
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Red);
        if (alliance == Alliance.Red)
            return this.redRectangle;
        else
            return this.blueRectangle;
    }
}