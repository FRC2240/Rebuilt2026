package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public class IntakeConstants {
  public static final int INTAKE_MOTOR_ID = 54; 
  public static final int PIVOT_MOTOR_ID = 55;

  public static final Angle EXTENDED_POSITION = Degrees.of(40); // TBD
  public static final AngularVelocity INTAKE_VELOCITY = RotationsPerSecond.of(-5); // TBD
}
