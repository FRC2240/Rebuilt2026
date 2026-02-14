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
    // Maximum robot speed
    public static final LinearVelocity MAX_SPEED = TunerConstants.kSpeedAt12Volts;
    public static final AngularVelocity MAX_ANGULAR_RATE = RotationsPerSecond.of(1);

    // Maximum robot speed when in slow mode
    public static final LinearVelocity MAX_SLOW_SPEED = MAX_SPEED.div(4);
    public static final AngularVelocity MAX_SLOW_ANGULAR_RATE = MAX_ANGULAR_RATE.div(3);

    public static final LinearAcceleration MAX_WHEEL_ACCELERATION = MetersPerSecondPerSecond.of(8);
    public static final LinearAcceleration MAX_WHEEL_DECELERATION = MetersPerSecondPerSecond.of(9);

    // PID controllers for autonomous driving
    public static final PIDController TRANSLATION_PID_CONTROLLER = new PIDController(5, 0, 1);
    public static final PIDController ROTATION_PID_CONTROLLER = new PIDController(5, 0, 0);

    public static final Distance TRANSLATION_FINISHED_THRESHOLD = Inches.of(2);
    public static final Angle ROTATION_FINISHED_THRESHOLD = Degrees.of(2);

    // Override conditions for autonomous driving
    public static final double CONTROLLER_OVERRIDE_THRESHOLD = 0.2;
    public static final double CONTROLLER_OVERRIDE_TIMEOUT = 0.3;
    public static final double CONTROLLER_DEADBAND = 0.1;
}
