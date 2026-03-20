package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;

public class SwerveLimiter {
    private final Drivetrain drivetrain;
    private ChassisSpeeds currentChassisSpeeds = new ChassisSpeeds();
    private double previousTime = Timer.getFPGATimestamp();

    /**
     * Constructs a new SwerveLimiter instance.
     * Requires a drivetrain to preform kinematics to get wheel speeds from chassis
     * speeds
     */
    public SwerveLimiter(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    /**
     * Calculates a new ChassisSpeeds proportional to the input speeds that respects
     * the max acceleration and deceleration of the wheels.
     */
    public ChassisSpeeds calculate(ChassisSpeeds targetSpeeds) {
        double currentTime = Timer.getFPGATimestamp();
        if (currentTime <= previousTime)
            return currentChassisSpeeds;
        Time dt = Seconds.of(Math.min(currentTime - previousTime, 0.2)); // Cap the time difference
        previousTime = currentTime;

        SwerveModuleState[] targetStates = drivetrain.getKinematics().toSwerveModuleStates(targetSpeeds);
        SwerveModuleState[] currentStates = drivetrain.getKinematics().toSwerveModuleStates(currentChassisSpeeds);

        double minFactor = 1.0;
        double maxAcceleratingDeltaV = DriveConstants.MAX_WHEEL_ACCELERATION.times(dt).in(MetersPerSecond);
        double maxDeceleratingDeltaV = DriveConstants.MAX_WHEEL_DECELERATION.times(dt).in(MetersPerSecond);

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