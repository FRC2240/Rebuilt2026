package frc.robot.subsystems.Shooter;

import edu.wpi.first.units.measure.Angle;

public class ShooterState {

    Angle angle;
    double speed;
    
    public ShooterState(Angle ang, double spd) {
        angle = ang;
        speed = spd;
    }

    public void add(ShooterState other) {
        angle = angle.plus(other.getAngle());
        speed += other.getSpeed();
    }

    public Angle getAngle() {
        return angle;
    }

    public double getSpeed() {
        return speed;
    }
}
