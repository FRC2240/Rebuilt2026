package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public class IntakeConstants {
  public static final int INTAKE_MOTOR_ID = 54; 
  public static final int PIVOT_MOTOR_ID = 55;
  public static final int INTAKE_MOTOR_FOLLOWER_ID = 62;

  public static final Angle PIVOT_EXTENDED_POSITION = Rotations.of(-10); 
  public static final Angle PIVOT_RAMP_POSITION = Rotations.of(-4.2);
  public static final AngularVelocity INTAKE_VELOCITY = RotationsPerSecond.of(-60); 
}