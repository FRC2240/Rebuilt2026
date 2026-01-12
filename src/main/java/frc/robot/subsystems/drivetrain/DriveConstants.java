package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.generated.TunerConstants;

public class DriveConstants {
    public static LinearVelocity MAX_SPEED = TunerConstants.kSpeedAt12Volts;
    public static AngularVelocity MAX_ANGULAR_RATE = RotationsPerSecond.of(0.75);
}
