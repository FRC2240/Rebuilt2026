package frc.robot.utils;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class FieldSimulation extends SubsystemBase {
  private static class Ball {
    LinearVelocity vX;
    LinearVelocity vY;
    LinearVelocity vZ; // Up / Down
    Distance x;
    Distance y;
    Distance z;

    public Ball(Distance x, Distance y, Distance z, LinearVelocity vx, LinearVelocity vy, LinearVelocity vz) {
      this.x = x;
      this.y = y;
      this.z = z;
      this.vX = vx;
      this.vY = vy;
      this.vZ = vz;
    }

    public void update(Time deltaTime) {
      // Update the Z velocity with acceleration due to gravity
      vZ = vZ.minus(MetersPerSecondPerSecond.of(9.8).times(deltaTime));

      // Use Explicit Euler method to update positions based off veloicity
      x = x.plus(vX.times(deltaTime));
      y = y.plus(vY.times(deltaTime));
      z = z.plus(vZ.times(deltaTime));
    }

    public boolean isFailed() {
      // Hit the ground
      if (this.z.compareTo(Meters.zero()) < 0)
        return true;
      return false;
    }
  }

  private final StructArrayPublisher<Translation3d> publisher;
  private double lastTime = Timer.getFPGATimestamp();
  private List<Ball> balls = new ArrayList<>();

  public FieldSimulation() {
    this.publisher = NetworkTableInstance
        .getDefault()
        .getStructArrayTopic("balls", Translation3d.struct)
        .publish();
  }

  public void shootWithRobotVelocity(Drivetrain drivetrain, Rotation2d shooterPitch, LinearVelocity shooterVelocity) {
    ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
        drivetrain.getState().Speeds,
        drivetrain.getHeading());

    double ballHorizontalSpeed = shooterVelocity.in(MetersPerSecond) * shooterPitch.getCos();
    double ballVerticalSpeed = shooterVelocity.in(MetersPerSecond) * shooterPitch.getSin();

    double ballVxRelative = ballHorizontalSpeed * drivetrain.getHeading().getCos();
    double ballVyRelative = ballHorizontalSpeed * drivetrain.getHeading().getSin();

    double finalVx = ballVxRelative + fieldSpeeds.vxMetersPerSecond;
    double finalVy = ballVyRelative + fieldSpeeds.vyMetersPerSecond;
    double finalVz = ballVerticalSpeed;

    balls.add(new Ball(
        drivetrain.getTranslation().getMeasureX(),
        drivetrain.getTranslation().getMeasureY(),
        Meters.of(0.5), // Assuming shooter height off ground
        MetersPerSecond.of(finalVx),
        MetersPerSecond.of(finalVy),
        MetersPerSecond.of(finalVz)));
  }

  @Override
  public void simulationPeriodic() {
    Time deltaTime = Seconds.of(Timer.getFPGATimestamp() - lastTime);
    this.lastTime = Timer.getFPGATimestamp();
    for (int i = 0; i < balls.size(); i++) {
      Ball ball = balls.get(i);
      ball.update(deltaTime);
      if (ball.isFailed()) {
        balls.remove(i);
        i--;
      }
    }

    Translation3d[] arr = balls.stream().map(ball -> new Translation3d(ball.x, ball.y, ball.z))
        .toArray(Translation3d[]::new);
    publisher.set(arr);
  }
}