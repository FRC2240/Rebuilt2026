package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;

public class ShooterConstants {

    public static final int FLYWHEEL_MOTOR_ID = 97;
    public static final int ANGLE_MOTOR_ID = 90;
    public static final int LOADER_MOTOR_ID = 83;

    public static Angle MIN_ANGLE = Angle.ofBaseUnits(14, Rotations);
    public static Angle MAX_ANGLE = Angle.ofBaseUnits(50, Rotations);

}
