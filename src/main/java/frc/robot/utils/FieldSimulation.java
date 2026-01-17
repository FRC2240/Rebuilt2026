package frc.robot.utils;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FieldSimulation extends SubsystemBase {
    private static class Ball {
        LinearVelocity vX;
        LinearVelocity vY;
        LinearVelocity vZ; // Up / Down
        Distance x;
        Distance y;
        Distance z;

        public Ball(Distance x, Distance y, Distance z, LinearVelocity velocity, Rotation2d heading, Rotation2d pitch) {
            this.x = x;
            this.y = y;
            this.z = z;

            this.vZ = velocity.times(pitch.getCos());
            LinearVelocity groundSpeed = velocity.times(pitch.getSin());
            this.vX = groundSpeed.times(heading.getCos());
            this.vY = groundSpeed.times(heading.getSin());
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

    public void addBall(Translation3d position, LinearVelocity velocity, Rotation2d heading,
            Rotation2d pitch) {
        balls.add(new Ball(
                position.getMeasureX(),
                position.getMeasureY(),
                position.getMeasureZ(),
                velocity,
                heading,
                pitch));
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