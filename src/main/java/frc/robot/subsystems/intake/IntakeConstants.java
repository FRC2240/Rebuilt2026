package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;

public class IntakeConstants {
  public static final int INTAKE_MOTOR_ID = 54; 
  public static final int INTAKE_MOTOR_FOLLOWER_ID = 59;

  public static final int PIVOT_MOTOR_ID = 55;
  public static final int PIVOT_TOLLERANCE = 1;

  public static final Current STATOR_CURRENT_LIMIT = Amps.of(85);
  public static final Angle PIVOT_EXTENDED_POSITION = Rotations.of(-10.495); 
  public static final Angle PIVOT_RAMP_POSITION = Rotations.of(-1.0);
  public static final Angle PIVOT_RAMP_BOTTOM_POSITION = Rotations.of(-8.5);
  public static final AngularVelocity INTAKE_VELOCITY = RotationsPerSecond.of(-60); 
}