package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;

public class ShooterConstants {

    public static final int FLYWHEEL_LEFT_MOTOR_ID = 97;
    public static final int FLYWHEEL_RIGHT_MOTOR_ID = 90;
    public static final int LAUNCH_LEFT_MOTOR_ID = 83;
    public static final int LAUNCH_RIGHT_MOTOR_ID = 76;

    // There is only a specific range or band that we can shoot from.
    // This is becasue too close to hub and it will hit the hub instead of going in and too far will be outside of field
    // or will be past the motors maximum output.
    // Ideally, the MIN output should be done at the abnds inner radius and the MAX at its out radius/edge of the field.
    // The output should be chosen based off of distance(radius).
    // These only apply when shooting. Setting to zero still works bc we are turning it off(aka not shooting)
    public static final AngularVelocity MAX_MOTOR_OUTPUT = AngularVelocity.ofBaseUnits(0.89, RotationsPerSecond);
    public static final AngularVelocity MIN_MOTOR_OUTPUT = AngularVelocity.ofBaseUnits(0.15, RotationsPerSecond);
    public static final AngularVelocity LAUNCH_MOTOR_OUTPUT = AngularVelocity.ofBaseUnits(60, RotationsPerSecond);
    public static final AngularVelocity PASSING_OUTPUT = AngularVelocity.ofBaseUnits(30, RotationsPerSecond);
}
