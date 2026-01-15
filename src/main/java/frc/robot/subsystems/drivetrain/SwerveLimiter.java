package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;

public class SwerveLimiter {
  private final Drivetrain drivetrain;
  private ChassisSpeeds currentChassisSpeeds = new ChassisSpeeds();
  private double previousTime = Timer.getFPGATimestamp();
  private final LinearAcceleration maxAcceleration;
  private final LinearAcceleration maxDeceleration;

  public SwerveLimiter(Drivetrain drivetrain, LinearAcceleration maxWheelAcceleration, LinearAcceleration maxWheelDeceleration) {
    this.drivetrain = drivetrain;
    this.maxAcceleration = maxWheelAcceleration;
    this.maxDeceleration = maxWheelDeceleration;
  }

  public ChassisSpeeds calculate(ChassisSpeeds targetSpeeds) {
    double currentTime = Timer.getFPGATimestamp();
    if (currentTime <= previousTime) return currentChassisSpeeds; 
    Time dt = Seconds.of(currentTime - previousTime);
    previousTime = currentTime;

    SwerveModuleState[] targetStates = drivetrain.getKinematics().toSwerveModuleStates(targetSpeeds);
    SwerveModuleState[] currentStates = drivetrain.getKinematics().toSwerveModuleStates(currentChassisSpeeds);

    double minFactor = 1.0;
    double maxAcceleratingDeltaV = maxAcceleration.times(dt).in(MetersPerSecond);
    double maxDeceleratingDeltaV = maxDeceleration.times(dt).in(MetersPerSecond);

    for (int i = 0; i < 4; i++) {
      double wheelV = targetStates[i].speedMetersPerSecond;
      double deltaV = wheelV - currentStates[i].speedMetersPerSecond;
      double allowedDeltaV = (deltaV > 0) ? maxAcceleratingDeltaV : maxDeceleratingDeltaV;

      if (Math.abs(deltaV) > allowedDeltaV) {
        minFactor = Math.min(minFactor, Math.abs(allowedDeltaV / deltaV));
      }
    }

    ChassisSpeeds deltaChassis = targetSpeeds.minus(currentChassisSpeeds);
    currentChassisSpeeds = currentChassisSpeeds.plus(deltaChassis.times(minFactor));

    return currentChassisSpeeds;
  }
}