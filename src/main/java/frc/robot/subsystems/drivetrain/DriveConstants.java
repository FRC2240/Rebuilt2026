package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.generated.TunerConstants;

public class DriveConstants {
    public static LinearVelocity MAX_SPEED = TunerConstants.kSpeedAt12Volts;
    public static AngularVelocity MAX_ANGULAR_RATE = RotationsPerSecond.of(0.75);

    public static LinearVelocity MAX_SLOW_SPEED = MAX_SPEED.div(4);
    public static AngularVelocity MAX_SLOW_ANGULAR_RATE = MAX_ANGULAR_RATE.div(3);

    public static LinearAcceleration MAX_WHEEL_ACCELERATION = MetersPerSecondPerSecond.of(14);
    public static LinearAcceleration MAX_WHEEL_DECELERATION = MetersPerSecondPerSecond.of(50);

    public static PIDController TRANSLATION_PID_CONTROLLER = new PIDController(5, 0, 1);
    public static PIDController ROTATION_PID_CONTROLLER = new PIDController(5, 0, 0);

    public static Distance TRANSLATION_FINISHED_THRESHOLD = Inches.of(2);
    public static Angle ROTATION_FINISHED_THRESHOLD = Degrees.of(2);

    // Override conditions for autonomous driving
    public static double CONTROLLER_OVERRIDE_THRESHOLD = 0.2;
    public static double CONTROLLER_OVERRIDE_TIMEOUT = 0.3;

    public static double CONTROLLER_DEADBAND = 0.1;
}
