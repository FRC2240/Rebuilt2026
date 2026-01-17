package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

import static edu.wpi.first.units.Units.Degrees;

public class VisionConstants {
  public static Transform3d CAMERA_0_POS = new Transform3d(0.31115, 0.2921, 0.20955,
      new Rotation3d(Degrees.of(0), Degrees.of(-30), Degrees.of(-20)));
  public static Transform3d CAMERA_1_POS = new Transform3d(0.31115, -0.2921, 0.20955,
      new Rotation3d(Degrees.of(0), Degrees.of(-30), Degrees.of(20)));

  public static Transform3d CAMERA_LL_POS = new Transform3d(-0.216373, 0, 0.595738,
      new Rotation3d(Degrees.of(0), Degrees.of(-30), Degrees.of(0)));

  public static double MAX_UNCERTAINTY = 0.3; // TBD
  public static double MAX_Z_ERROR = 0.2; // TBD

  // Standard deviation coefficents, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static double LINEAR_STDEV_COEFF = 0.02; // Meters
  public static double ANGULAR_STDEV_COEFF = 0.06; // Radians

  // Multipliers to apply for MegaTag 2 observations
  public static double LINEAR_STDEV_MEGATAG_2_COEFF = 0.5; // More stable than full 3D solve
  public static double ANGULAR_STDEV_MEGATAG_2_COEFF = Double.POSITIVE_INFINITY; // No rotation data available
}
